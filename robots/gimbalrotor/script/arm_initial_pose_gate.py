#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tell the arm controller spawner when the simulated arm has settled at its initial pose.

In gazebo the arm joints are unactuated until servo_bridge has loaded their position
controllers, so the arm falls under gravity for the first second. JointTrajectoryController
holds whatever pose it sees when it starts, so starting it during that fall freezes a
random, possibly self-intersecting pose.

servo_bridge sends each init_value only once, as soon as its command topic has any
subscriber, so the joint controller can miss it and keep holding the fallen pose. This node
therefore keeps commanding init_value through ``<servo_group>_ctrl`` (the servo_bridge
input the trajectory controller also uses) until every servo controller of the group is
running and every joint has stayed within ``~tolerance`` of its init_value for
``~settle_time``. It then stops commanding and publishes True (latched) on ``~ready``, which
the spawner waits for with ``--wait-for``.

Parameters
----------
~servo_group : str
    Servo group in the servo_bridge config (default: joints).
~tolerance : float
    Allowed distance from init_value [rad] (default: 0.15). It only has to tell a settled arm from
    a falling one (1-3 rad away); arm_joint2 sags ~0.05 rad under gravity with the group gains.
~settle_time : float
    Sim time the joints must stay within tolerance [s] (default: 0.5).
~timeout : float
    Sim time after which a still unsettled arm is reported every second [s] (default: 30.0).
"""

from controller_manager_msgs.srv import ListControllers
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def load_servo_targets(group):
    """Return the joint names and init values of a servo group.

    Parameters
    ----------
    group : str
        Servo group name under ``servo_controller``.

    Returns
    -------
    list of tuple
        ``(controller_key, joint_name, init_value)`` for each servo.
    """
    group_params = rospy.get_param("servo_controller/" + group)
    group_init = group_params.get("simulation", {}).get("init_value")
    targets = []
    for key in sorted(k for k in group_params if k.startswith("controller")):
        servo = group_params[key]
        init_value = servo.get("simulation", {}).get("init_value", group_init)
        if init_value is None:
            raise RuntimeError("servo_controller/{}/{} has no simulation/init_value".format(group, key))
        targets.append((key, servo["name"], float(init_value)))
    return targets


def main():
    rospy.init_node("arm_initial_pose_gate")
    group = rospy.get_param("~servo_group", "joints")
    tolerance = rospy.get_param("~tolerance", 0.15)
    settle_time = rospy.get_param("~settle_time", 0.5)
    timeout = rospy.get_param("~timeout", 30.0)

    targets = load_servo_targets(group)
    ready_pub = rospy.Publisher("~ready", Bool, queue_size=1, latch=True)
    ready_pub.publish(Bool(data=False))
    command_pub = rospy.Publisher(group + "_ctrl", JointState, queue_size=1)
    command = JointState(name=[joint for _, joint, _ in targets], position=[value for _, _, value in targets])

    namespace = rospy.get_namespace().rstrip("/")
    expected = {"{}/servo_controller/{}/{}/simulation".format(namespace, group, key) for key, _, _ in targets}
    list_srv_name = "controller_manager/list_controllers"
    rospy.wait_for_service(list_srv_name)
    list_controllers = rospy.ServiceProxy(list_srv_name, ListControllers)

    state = {"positions": None}

    def joint_cb(msg):
        state["positions"] = dict(zip(msg.name, msg.position))

    rospy.Subscriber("joint_states", JointState, joint_cb, queue_size=1)

    rate = rospy.Rate(20)
    started = rospy.get_time()
    settled_since = None
    while not rospy.is_shutdown():
        now = rospy.get_time()
        command.header.stamp = rospy.Time.now()
        command_pub.publish(command)
        running = {c.name for c in list_controllers().controller if c.state == "running"}
        missing = sorted(expected - running)
        errors = {}
        positions = state["positions"]
        if positions is not None:
            for _, joint, init_value in targets:
                if joint in positions:
                    errors[joint] = abs(positions[joint] - init_value)
        all_close = len(errors) == len(targets) and max(errors.values()) <= tolerance

        if not missing and all_close:
            if settled_since is None:
                settled_since = now
            if now - settled_since >= settle_time:
                ready_pub.publish(Bool(data=True))
                rospy.loginfo("[arm gate] arm settled at the initial pose (max error %.3f rad), starting the arm controller",
                              max(errors.values()))
                break
        else:
            settled_since = None

        if now - started > timeout:
            rospy.logerr_throttle(1.0, "[arm gate] arm not ready after %.1f s: controllers not running %s, joint errors %s",
                                  now - started, missing, {k: round(v, 3) for k, v in errors.items()})
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()
