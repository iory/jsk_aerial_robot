// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <spinal/DesireCoord.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

namespace aerial_robot_navigation
{
class GimbalrotorNavigator : public BaseNavigator
{
public:
  GimbalrotorNavigator();
  ~GimbalrotorNavigator()
  {
  }

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du) override;

  void update() override;

  /* Takeoff interlock: the gimbals have to be commanded to zero through the topic gimbals_zero (a human
     step, so that someone looks at them) and measured within takeoff_gimbal_tolerance of zero, since
     arming; a takeoff with the gimbals left tilted by the previous flight tipped the robot over at liftoff
     (2026-09-18). */
  bool takeoffAllowed() override;

private:
  ros::Publisher gimbal_zero_pub_;
  ros::Subscriber gimbals_zero_sub_;
  void gimbalsZeroCallback(const std_msgs::EmptyConstPtr& msg);
  bool gimbals_zeroed_;
  double takeoff_gimbal_tolerance_;  // [rad]
  bool takeoff_gimbal_check_;
  ros::Publisher target_baselink_rpy_pub_;
  ros::Subscriber final_target_baselink_rot_sub_, final_target_baselink_rpy_sub_;

  void baselinkRotationProcess();
  void rosParamInit() override;
  void targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg);
  void targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void reset() override;

  /* target baselink rotation */
  double prev_rotation_stamp_;
  tf::Quaternion curr_target_baselink_rot_, final_target_baselink_rot_;
  bool eq_cog_world_;

  /* rosparam */
  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;
};
};  // namespace aerial_robot_navigation
