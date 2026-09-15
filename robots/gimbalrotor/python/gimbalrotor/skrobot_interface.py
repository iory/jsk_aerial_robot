#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scikit-robot interface for gimbalrotor (grape_with_arm).

Example
-------
>>> import numpy as np
>>> from gimbalrotor.skrobot_interface import GimbalrotorROSRobotInterface
>>> ri = GimbalrotorROSRobotInterface()
>>> robot = ri.robot
>>> ri.start()     # motor arming
>>> ri.takeoff()
>>> ri.move_to([1.0, 0.0, 1.2])  # world frame, origin: estimator start
>>> robot.arm_joint2_joint1.joint_angle(-0.5)
>>> ri.angle_vector(robot.angle_vector(), 3.0)
>>> ri.wait_interpolation()
>>> ri.start_grasp()   # close the gripper
>>> ri.gripper_width()
>>> ri.stop_grasp()    # open the gripper
>>> ri.move_to([0.0, 0.0, 1.2])
>>> ri.land()
"""

import xml.etree.ElementTree as ET

from aerial_robot_base.robot_interface import RobotInterface
import control_msgs.msg
import numpy as np
import rospy
from skrobot.coordinates import Coordinates
from skrobot.coordinates.math import quaternion2matrix
from skrobot.coordinates.math import xyzw2wxyz
from skrobot.interfaces.ros.base import ROSRobotInterfaceBase
from skrobot.models.urdf import RobotModelFromURDF


class GripperMimicModel(object):
    """Relation between the gripper drive joint and the finger opening.

    The fingers are prismatic joints that mimic the drive joint in the URDF
    (``position = multiplier * drive_angle + offset``). The gripper width is
    defined as the sum of the finger joint positions, so it is 0 at the zero
    position of the fingers in the URDF, and larger values are assumed to be
    more open.

    Parameters
    ----------
    urdf : str
        URDF string.
    drive_joint_name : str
        Name of the joint that drives the fingers.
    """

    def __init__(self, urdf, drive_joint_name):
        joints = {j.attrib['name']: j
                  for j in ET.fromstring(urdf).findall('joint')}
        if drive_joint_name not in joints:
            raise RuntimeError(
                'joint {} is not found in the URDF'.format(drive_joint_name))
        self.drive_joint_name = drive_joint_name
        lower, upper = self._limit(joints[drive_joint_name])
        self.slope = 0.0
        self.offset = 0.0
        self.finger_joint_names = []
        for name, joint in joints.items():
            mimic = joint.find('mimic')
            if mimic is None or mimic.attrib.get('joint') != drive_joint_name:
                continue
            multiplier = float(mimic.attrib.get('multiplier', 1.0))
            offset = float(mimic.attrib.get('offset', 0.0))
            if multiplier == 0.0:
                continue
            # every mimic joint with limits narrows the range of the drive
            if joint.attrib.get('type') in ('revolute', 'prismatic'):
                mimic_lower, mimic_upper = self._limit(joint)
                bounds = sorted([(mimic_lower - offset) / multiplier,
                                 (mimic_upper - offset) / multiplier])
                lower = max(lower, bounds[0])
                upper = min(upper, bounds[1])
            if joint.attrib.get('type') == 'prismatic':
                self.slope += multiplier
                self.offset += offset
                self.finger_joint_names.append(name)
        if len(self.finger_joint_names) == 0 or self.slope == 0.0:
            raise RuntimeError(
                'no prismatic finger joint mimics {}'.format(drive_joint_name))
        if lower > upper:
            raise RuntimeError(
                'the limits of {} and its fingers do not overlap'.format(
                    drive_joint_name))
        self.angle_range = (lower, upper)
        self.width_range = tuple(sorted(
            [self.width(lower), self.width(upper)]))

    @staticmethod
    def _limit(joint):
        limit = joint.find('limit')
        if limit is None or 'lower' not in limit.attrib \
           or 'upper' not in limit.attrib:
            raise RuntimeError(
                'joint {} has no lower/upper limit'.format(
                    joint.attrib['name']))
        return float(limit.attrib['lower']), float(limit.attrib['upper'])

    def width(self, angle):
        """Return the gripper width [m] for a drive joint angle [rad]."""
        return self.slope * angle + self.offset

    def angle(self, width):
        """Return the drive joint angle [rad] for a gripper width [m]."""
        return (width - self.offset) / self.slope


class GimbalrotorROSRobotInterface(ROSRobotInterfaceBase):
    """Robot interface for the flight, the arm and the gripper of gimbalrotor.

    The arm and the gripper are controlled through the FollowJointTrajectory
    action of ``<namespace>/arm/arm_controller``; the gripper drive joint is
    one of its joints. The flight commands are sent through
    ``aerial_robot_base.robot_interface.RobotInterface``.

    Parameters
    ----------
    robot : skrobot.model.RobotModel or None
        Robot model. If None, it is created from
        ``<namespace>/robot_description``.
    namespace : str
        Robot namespace (default: ``gimbalrotor``).
    arm_controller_ns : str
        Namespace of the arm controller relative to ``namespace``.
    gripper_joint_name : str or None
        Joint of ``arm_controller`` that drives the gripper fingers. If None,
        the joint whose prismatic mimic joints are the fingers in the URDF.
    sync_base_pose : bool
        If True, ``update_robot_state`` also moves the root of ``robot`` so
        that the baselink matches ``<namespace>/uav/baselink/odom``.
    **kwargs
        Passed to ``ROSRobotInterfaceBase``.
    """

    def __init__(self, robot=None, namespace='gimbalrotor',
                 arm_controller_ns='arm/arm_controller',
                 gripper_joint_name=None,
                 sync_base_pose=True, **kwargs):
        if not rospy.core.is_initialized():
            rospy.init_node('gimbalrotor_skrobot_interface', anonymous=True,
                            disable_signals=True)
        description_param = '/{}/robot_description'.format(namespace)
        if not rospy.has_param(description_param):
            raise RuntimeError(
                'rosparam {} is not found. '
                'Is bringup.launch running?'.format(description_param))
        urdf = rospy.get_param(description_param)
        if robot is None:
            robot = RobotModelFromURDF(urdf=urdf)
        self.baselink_name = self._baselink_name(urdf)
        self.arm_controller_ns = arm_controller_ns
        self.sync_base_pose = sync_base_pose

        self.flight = RobotInterface(robot_ns='/' + namespace)
        if self.flight.base_odom is None or self.flight.cog_odom is None:
            raise RuntimeError(
                'odometry of /{} is not received'.format(namespace))

        super(GimbalrotorROSRobotInterface, self).__init__(
            robot, namespace=namespace, **kwargs)
        if not self.joint_action_enable:
            raise RuntimeError(
                'arm controller /{}/{}/follow_joint_trajectory is not '
                'available'.format(namespace, arm_controller_ns))
        self.gripper = self._gripper_model(urdf, gripper_joint_name)

    def _gripper_model(self, urdf, gripper_joint_name):
        joint_names = self.arm_controller['joint_names']
        if gripper_joint_name is not None:
            if gripper_joint_name not in joint_names:
                raise RuntimeError(
                    '{} is not a joint of arm_controller {}'.format(
                        gripper_joint_name, joint_names))
            return GripperMimicModel(urdf, gripper_joint_name)
        models = []
        for name in joint_names:
            try:
                models.append(GripperMimicModel(urdf, name))
            except RuntimeError:
                continue
        if len(models) != 1:
            raise RuntimeError(
                'expected one gripper drive joint in arm_controller {}, '
                'found {}'.format(joint_names,
                                  [m.drive_joint_name for m in models]))
        return models[0]

    @staticmethod
    def _baselink_name(urdf):
        """Return the name in ``<baselink name="..."/>`` of the URDF."""
        element = ET.fromstring(urdf).find('baselink')
        if element is None or 'name' not in element.attrib:
            raise RuntimeError('<baselink> is not found in robot_description')
        return element.attrib['name']

    @property
    def arm_controller(self):
        joint_names = rospy.get_param(
            '/{}/{}/joints'.format(self.namespace, self.arm_controller_ns))
        return dict(
            controller_type='arm_controller',
            controller_action=self.arm_controller_ns
            + '/follow_joint_trajectory',
            controller_state=self.arm_controller_ns + '/state',
            action_type=control_msgs.msg.FollowJointTrajectoryAction,
            joint_names=joint_names,
        )

    def default_controller(self):
        return [self.arm_controller]

    # ---------------------------------------------------------------------
    # state
    # ---------------------------------------------------------------------
    @property
    def flight_state(self):
        """Flight state (see ``RobotInterface.*_STATE``)."""
        return self.flight.getFlightState()

    def cog_position(self):
        """Return the CoG position in the world frame [m]."""
        return np.array(self.flight.getCogPos())

    def yaw(self):
        """Return the yaw angle of the baselink in the world frame [rad]."""
        return self.flight.getBaseRPY()[2]

    def baselink_coords(self):
        """Return the baselink pose in the world frame.

        Returns
        -------
        skrobot.coordinates.Coordinates
            Pose of the baselink.
        """
        rot = quaternion2matrix(xyzw2wxyz(np.array(self.flight.getBaseRot())))
        return Coordinates(pos=np.array(self.flight.getBasePos()), rot=rot)

    def update_robot_state(self, wait_until_update=False):
        ret = super(GimbalrotorROSRobotInterface, self).update_robot_state(
            wait_until_update=wait_until_update)
        if ret and self.sync_base_pose:
            baselink = getattr(self.robot, self.baselink_name)
            root_to_baselink = self.robot.root_link.copy_worldcoords() \
                .inverse_transformation() \
                .transform(baselink.copy_worldcoords())
            world_to_root = self.baselink_coords().transform(
                root_to_baselink.inverse_transformation())
            self.robot.newcoords(world_to_root)
        return ret

    # ---------------------------------------------------------------------
    # flight
    # ---------------------------------------------------------------------
    def _wait_flight_state(self, target_state, timeout):
        start = rospy.get_time()
        while not rospy.is_shutdown():
            if self.flight_state == target_state:
                return True
            if rospy.get_time() - start > timeout:
                rospy.logwarn('[%s] timeout (%.1f s) for flight state %d, '
                              'current: %s', rospy.get_name(), timeout,
                              target_state, self.flight_state)
                return False
            rospy.sleep(0.05)
        return False

    def start(self, timeout=2.0):
        """Start the motors (ARM_OFF -> ARM_ON).

        Returns
        -------
        bool
            True if the motors are started or the robot already hovers.
        """
        if self.flight_state in (self.flight.ARM_ON_STATE,
                                 self.flight.HOVER_STATE):
            return True
        if self.flight_state != self.flight.ARM_OFF_STATE:
            rospy.logwarn('[%s] can not start motors in flight state %s',
                          rospy.get_name(), self.flight_state)
            return False
        self.flight.start(sleep=0.0)
        return self._wait_flight_state(self.flight.ARM_ON_STATE, timeout)

    def takeoff(self, timeout=30.0):
        """Take off (ARM_ON -> HOVER).

        Returns
        -------
        bool
            True if the robot hovers.
        """
        if self.flight_state == self.flight.HOVER_STATE:
            return True
        if self.flight_state != self.flight.ARM_ON_STATE:
            rospy.logwarn('[%s] can not take off in flight state %s. '
                          'call start() first', rospy.get_name(),
                          self.flight_state)
            return False
        self.flight.takeoff()
        return self._wait_flight_state(self.flight.HOVER_STATE, timeout)

    def land(self, wait=True, timeout=20.0):
        """Land (HOVER -> ARM_OFF).

        Returns
        -------
        bool
            True if the robot landed (or the command is sent when
            ``wait`` is False).
        """
        if self.flight_state != self.flight.HOVER_STATE:
            rospy.logwarn('[%s] can not land in flight state %s',
                          rospy.get_name(), self.flight_state)
            return False
        self.flight.land()
        if not wait:
            return True
        return self._wait_flight_state(self.flight.ARM_OFF_STATE, timeout)

    def halt(self):
        """Stop the motors immediately."""
        self.flight.halt()

    def force_landing(self):
        self.flight.forceLanding()

    def move_to(self, pos, yaw=None, wait=True, pos_thresh=0.1,
                vel_thresh=0.05, yaw_thresh=0.1, timeout=30.0):
        """Move the CoG to a position in the world frame.

        The world frame is defined by the state estimator; with LIO, its
        origin is the pose where the estimator started.

        Parameters
        ----------
        pos : array_like
            Target CoG position [x, y, z] in the world frame [m].
        yaw : float or None
            Target yaw [rad]. If None, the current yaw is kept.
        wait : bool
            If True, block until the robot converges or ``timeout``.
        pos_thresh, vel_thresh, yaw_thresh : float
            Convergence thresholds [m], [m/s], [rad].
        timeout : float
            Timeout [s].

        Returns
        -------
        bool
            True if converged (or the command is sent when ``wait`` is
            False).
        """
        pos = np.asarray(pos, dtype=np.float64)
        if pos.shape != (3,):
            raise ValueError('pos must be [x, y, z], but given {}'.format(pos))
        if not wait:
            timeout = -1
        if yaw is None:
            return self.flight.goPos(pos, pos_thresh=pos_thresh,
                                     vel_thresh=vel_thresh, timeout=timeout)
        return self.flight.goPosYaw(pos, yaw, pos_thresh=pos_thresh,
                                    vel_thresh=vel_thresh,
                                    yaw_thresh=yaw_thresh, timeout=timeout)

    def go_pos(self, x=0.0, y=0.0, z=0.0, yaw=0.0, **kwargs):
        """Move relative to the current pose.

        ``x`` and ``y`` are in the heading frame (rotated by the current yaw),
        ``z`` is along the world z axis.

        Parameters
        ----------
        x, y, z : float
            Relative displacement [m].
        yaw : float
            Relative yaw [rad].
        **kwargs
            Passed to ``move_to``.

        Returns
        -------
        bool
            Result of ``move_to``.
        """
        current_yaw = self.yaw()
        c = np.cos(current_yaw)
        s = np.sin(current_yaw)
        target = self.cog_position() + np.array(
            [c * x - s * y, s * x + c * y, z])
        target_yaw = np.arctan2(np.sin(current_yaw + yaw),
                                np.cos(current_yaw + yaw))
        return self.move_to(target, yaw=target_yaw, **kwargs)

    # ---------------------------------------------------------------------
    # gripper
    # ---------------------------------------------------------------------
    def gripper_width(self):
        """Return the current gripper width [m].

        The width is the sum of the finger joint positions in the URDF (see
        ``GripperMimicModel``), computed from the measured angle of the
        gripper drive joint. This does not change the joint angles of
        ``self.robot``.

        Returns
        -------
        float
            Current gripper width [m].
        """
        name = self.gripper.drive_joint_name
        if name not in self._received_joint_names:
            raise RuntimeError(
                'joint state of {} is not received from {}'.format(
                    name, self.joint_states_topic))
        index = self.robot_state['name'].index(name)
        return self.gripper.width(self.robot_state['position'][index])

    def move_gripper(self, width, time=1.0, wait=True):
        """Move the gripper to the target width.

        The target is sent to ``arm_controller`` as a trajectory of all its
        joints: the gripper drive joint moves to ``width`` and the other
        joints keep the positions the controller currently commands. This
        replaces a running arm trajectory, so do not call it while the arm is
        moving. ``self.robot`` is not changed.

        Parameters
        ----------
        width : float
            Target width [m], within ``self.gripper.width_range``.
        time : float
            Duration of the motion [s].
        wait : bool
            If True, wait until the controller finishes the trajectory.

        Returns
        -------
        bool or None
            If ``wait`` is True, whether the trajectory succeeded. It is
            False when a joint is out of the goal tolerance of
            ``arm_controller`` after the motion, e.g. the fingers are stopped
            by an object; the controller then keeps commanding the target, so
            the servo keeps pushing toward it. None if ``wait`` is False.
        """
        lower, upper = self.gripper.width_range
        if not lower <= width <= upper:
            raise ValueError(
                'width {} is out of range [{}, {}]'.format(
                    width, lower, upper))
        controller = self.arm_controller
        state = self.robot_state.get(controller['controller_state'])
        if state is None:
            raise RuntimeError(
                'state of arm_controller ({}) is not received'.format(
                    controller['controller_state']))
        model_joint_names = [j.name for j in self.robot.joint_list]
        av = np.zeros(len(model_joint_names))
        for name, position in zip(state.joint_names, state.desired.positions):
            av[model_joint_names.index(name)] = position
        av[model_joint_names.index(self.gripper.drive_joint_name)] = \
            self.gripper.angle(width)
        action = self.controller_table[controller['controller_type']][0]
        self.send_ros_controller(action, controller['joint_names'], 0.0,
                                 [(av, np.zeros(len(av)), time)])
        if not wait:
            return None
        action.wait_for_result()
        result = action.get_result()
        if result is None or result.error_code != \
           control_msgs.msg.FollowJointTrajectoryResult.SUCCESSFUL:
            rospy.logwarn('[%s] gripper motion did not succeed: %s',
                          rospy.get_name(),
                          None if result is None else result.error_string)
            return False
        return True

    def start_grasp(self, width=None, **kwargs):
        """Close the gripper to grasp an object.

        Parameters
        ----------
        width : float or None
            Target width [m]. If None, the minimum of
            ``self.gripper.width_range``. The servo keeps pushing toward it
            when an object stops the fingers, so for a soft object choose a
            width close to its size.
        **kwargs
            Passed to ``move_gripper``.

        Returns
        -------
        bool or None
            See ``move_gripper``. False typically means that an object stops
            the fingers; check ``gripper_width()``.
        """
        if width is None:
            width = self.gripper.width_range[0]
        return self.move_gripper(width, **kwargs)

    def stop_grasp(self, width=None, **kwargs):
        """Open the gripper to release an object.

        Parameters
        ----------
        width : float or None
            Target width [m]. If None, the maximum of
            ``self.gripper.width_range``.
        **kwargs
            Passed to ``move_gripper``.

        Returns
        -------
        bool or None
            See ``move_gripper``.
        """
        if width is None:
            width = self.gripper.width_range[1]
        return self.move_gripper(width, **kwargs)
