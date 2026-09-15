#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Emulate the servo torque on/off of spinal in gazebo.

On the real machine spinal receives ``servo/torque_enable`` (spinal/ServoTorqueCmd, indexed by the
servo id of servo_bridge), publishes ``servo/torque_states`` (spinal/ServoTorqueStates, one flag per
servo id, 1 Hz) and turns a servo back on when it receives a position command on
``servo/target_states``. This node gives gazebo the same interface, so that the torque services and
``torque_on()`` / ``torque_off()`` of the skrobot interface behave as on the real machine.

servo_bridge drives each gazebo joint with an ``effort_controllers/JointPositionController``
(``<ns>/servo_controller/<group>/<controller>/simulation``). Turning a servo off switches it to an
``effort_controllers/JointVelocityController`` commanding zero velocity, i.e. a viscous damper
(effort = -damping * velocity): the joint falls under gravity but does not swing freely, like a
back-driven servo gear. Turning it on switches back; JointPositionController holds the position it
starts at.

The damping [N m s/rad] is ``simulation/torque_off_damping`` of the servo or its group in the servo_bridge
config, or else the ``d`` gain of its simulation pid, which is known to be stable with the joint inertia.

Parameters
----------
~publish_rate : float
    Rate of ``servo/torque_states`` [Hz] (default: 1.0, as spinal). It is also published at every change.
"""

import threading

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import LoadController
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest
import rospy
from spinal.msg import ServoControlCmd
from spinal.msg import ServoTorqueCmd
from spinal.msg import ServoTorqueStates
from std_msgs.msg import Float64


class SimServo(object):
    """Gazebo controllers of one servo.

    Parameters
    ----------
    namespace : str
        Robot namespace with a leading slash.
    group : str
        Servo group name under ``servo_controller``.
    key : str
        Controller key of the servo, e.g. ``controller1``.
    joint : str
        Joint name.
    damping : float
        Viscous damping while the torque is off [N m s/rad].
    """

    def __init__(self, namespace, group, key, joint, damping):
        self.joint = joint
        base = "{}/servo_controller/{}/{}".format(namespace, group, key)
        self.position_controller = base + "/simulation"
        self.off_controller = base + "/simulation_torque_off"
        rospy.set_param(self.off_controller, {"type": "effort_controllers/JointVelocityController", "joint": joint,
                                              "pid": {"p": float(damping), "i": 0.0, "d": 0.0}})
        self.off_command_pub = rospy.Publisher(self.off_controller + "/command", Float64, queue_size=1, latch=True)


def torque_off_damping(servo_sim, group_sim):
    """Return the damping of a servo while its torque is off.

    Parameters
    ----------
    servo_sim : dict
        ``simulation`` parameters of the servo.
    group_sim : dict
        ``simulation`` parameters of its group.

    Returns
    -------
    float
        Damping [N m s/rad].
    """
    for params in (servo_sim, group_sim):
        if "torque_off_damping" in params:
            return float(params["torque_off_damping"])
    for params in (servo_sim, group_sim):
        if "pid" in params and "d" in params["pid"]:
            return float(params["pid"]["d"])
    raise RuntimeError("no simulation/torque_off_damping nor simulation/pid/d")


class ServoTorqueSim(object):
    def __init__(self):
        self.namespace = rospy.get_namespace().rstrip("/")
        self.lock = threading.Lock()
        self.servos = {}  # servo id -> SimServo
        for group, group_params in rospy.get_param("servo_controller").items():
            if not isinstance(group_params, dict):
                continue
            for key, servo in group_params.items():
                if key.startswith("controller") and isinstance(servo, dict) and "id" in servo and "name" in servo:
                    damping = torque_off_damping(servo.get("simulation", {}), group_params.get("simulation", {}))
                    self.servos[int(servo["id"])] = SimServo(self.namespace, group, key, servo["name"], damping)
        if not self.servos:
            raise RuntimeError("no servo is found in {}/servo_controller".format(self.namespace))
        self.enabled = [True] * (max(self.servos) + 1)

        manager = "controller_manager/"
        for name in ("list_controllers", "load_controller", "switch_controller"):
            rospy.wait_for_service(manager + name)
        self.list_controllers = rospy.ServiceProxy(manager + "list_controllers", ListControllers)
        self.load_controller = rospy.ServiceProxy(manager + "load_controller", LoadController)
        self.switch_controller = rospy.ServiceProxy(manager + "switch_controller", SwitchController)
        self.wait_position_controllers()
        for servo in self.servos.values():
            if not self.load_controller(servo.off_controller).ok:
                raise RuntimeError("failed to load {}".format(servo.off_controller))

        self.states_pub = rospy.Publisher("servo/torque_states", ServoTorqueStates, queue_size=1)
        rospy.Subscriber("servo/torque_enable", ServoTorqueCmd, self.torque_enable_cb, queue_size=10)
        rospy.Subscriber("servo/target_states", ServoControlCmd, self.target_states_cb, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~publish_rate", 1.0)), lambda _: self.publish_states())
        self.publish_states()
        rospy.loginfo("[servo torque sim] servo torque on/off is emulated for %d servos", len(self.servos))

    def wait_position_controllers(self):
        """Wait until servo_bridge has started the position controller of every servo."""
        expected = {servo.position_controller for servo in self.servos.values()}
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            running = {c.name for c in self.list_controllers().controller if c.state == "running"}
            if expected <= running:
                return
            rate.sleep()

    def publish_states(self):
        with self.lock:
            msg = ServoTorqueStates(torque_enable=[1 if e else 0 for e in self.enabled])
        self.states_pub.publish(msg)

    def switch(self, requests):
        """Switch the torque of servos.

        Parameters
        ----------
        requests : dict
            ``{servo id: enable}``.
        """
        with self.lock:
            changes = {i: e for i, e in requests.items() if i in self.servos and self.enabled[i] != e}
            if not changes:
                return
            on = [self.servos[i] for i, e in changes.items() if e]
            off = [self.servos[i] for i, e in changes.items() if not e]
            req = SwitchControllerRequest()
            req.start_controllers = [s.position_controller for s in on] + [s.off_controller for s in off]
            req.stop_controllers = [s.off_controller for s in on] + [s.position_controller for s in off]
            req.strictness = SwitchControllerRequest.STRICT
            if not self.switch_controller(req).ok:
                rospy.logerr("[servo torque sim] failed to switch the controllers %s", req)
                return
            for servo in off:
                servo.off_command_pub.publish(Float64(0.0))
            for i, e in changes.items():
                self.enabled[i] = e
            for i, e in sorted(changes.items()):
                rospy.loginfo("[servo torque sim] torque of %s (id %d) %s", self.servos[i].joint, i, "on" if e else "off")
        self.publish_states()

    def torque_enable_cb(self, msg):
        if len(msg.index) != len(msg.torque_enable):
            rospy.logerr("[servo torque sim] servo torque command has %d indices but %d torque flags",
                         len(msg.index), len(msg.torque_enable))
            return
        self.switch({i: bool(e) for i, e in zip(msg.index, msg.torque_enable)})

    def target_states_cb(self, msg):
        # spinal turns a servo back on with a position command
        with self.lock:
            off = [i for i in msg.index if i in self.servos and not self.enabled[i]]
        if off:
            self.switch({i: True for i in off})


def main():
    rospy.init_node("servo_torque_sim")
    ServoTorqueSim()
    rospy.spin()


if __name__ == "__main__":
    main()
