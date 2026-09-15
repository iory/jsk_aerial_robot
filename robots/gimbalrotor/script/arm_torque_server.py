#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Services to turn the servo torque of the arm joints on and off.

Advertises std_srvs/SetBool services (``data: true`` turns the torque on):

- ``~all``: every joint of the arm controller
- ``~<joint name>``: one joint

Turning a joint on first moves the arm controller target to the measured
positions (see ``gimbalrotor.servo_torque.ArmServoTorque``).

Parameters
----------
~robot_ns : str
    Robot namespace (default: gimbalrotor).
~arm_controller_ns : str
    Arm controller namespace relative to ``~robot_ns``
    (default: arm/arm_controller).
~timeout : float
    Time to wait for the torque state to follow a command [s] (default: 2.0).
"""

import rospy
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse

from gimbalrotor.servo_torque import ArmServoTorque


def main():
    rospy.init_node('arm_torque')
    robot_ns = rospy.get_param('~robot_ns', 'gimbalrotor')
    arm_controller_ns = rospy.get_param('~arm_controller_ns',
                                        'arm/arm_controller')
    timeout = rospy.get_param('~timeout', 2.0)
    torque = ArmServoTorque(robot_ns, arm_controller_ns)

    def handler(joint_names):
        def callback(req):
            action = 'on' if req.data else 'off'
            try:
                if req.data:
                    ok = torque.on(joint_names, timeout)
                else:
                    ok = torque.off(joint_names, timeout)
                states = torque.states()
            except (RuntimeError, ValueError, TypeError) as e:
                rospy.logerr('[arm torque] torque %s of %s failed: %s', action,
                             joint_names or 'all joints', e)
                return SetBoolResponse(success=False, message=str(e))
            message = ', '.join('{}: {}'.format(n, 'on' if on else 'off')
                                for n, on in states.items())
            if not ok:
                message = 'torque {} is not confirmed; {}'.format(
                    action, message)
            rospy.loginfo('[arm torque] torque %s of %s: %s', action,
                          joint_names or 'all joints', message)
            return SetBoolResponse(success=ok, message=message)
        return callback

    services = [rospy.Service('~all', SetBool, handler(None))]
    for name in torque.joint_names:
        services.append(rospy.Service('~' + name, SetBool, handler([name])))
    rospy.loginfo('[arm torque] services: %s',
                  ', '.join(s.resolved_name for s in services))
    rospy.spin()


if __name__ == '__main__':
    main()
