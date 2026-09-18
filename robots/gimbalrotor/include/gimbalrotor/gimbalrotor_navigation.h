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

  /* Flight supervisor (navigation/supervisor): the tiers of a production autopilot, from the mildest reaction
     up (ArduPilot CRASH_CHECK: attitude error 30 deg for 2 s -> disarm; PX4 FD_FAIL_R/P: 60 deg for 0.3 s ->
     termination). Runs every navigator loop while in flight.
       takeoff abort: TAKEOFF_STATE, lower than takeoff_abort_height above the ground and tilted more than
                      takeoff_abort_tilt -> motors off (the robot is over its legs; a flight of 2026-09-18 tipped
                      over at 4-7 cm height with 49 N of thrust because nothing stopped it)
       force landing: tilt above force_landing_tilt for force_landing_tilt_time, the xy error above
                      max_pos_error_xy, the z error (hover) above max_pos_error_z, speed above max_speed, higher
                      than max_height, or outside the geofence -> spinal FORCE_LANDING_CMD (level attitude and a
                      slow thrust ramp-down), as the existing sensor failsafe does
       halt:          tilt above halt_tilt, or below the ground while falling -> motors off (the crash is certain;
                      today's robot kept 31 N on while lying at 75 deg) */
  void supervise();

private:
  bool supervisor_enable_;
  double takeoff_abort_height_;     // [m] above the height at the takeoff command
  double takeoff_abort_tilt_;       // [rad]
  double force_landing_tilt_;       // [rad]
  double force_landing_tilt_time_;  // [s]
  double max_pos_error_xy_;         // [m]
  double max_pos_error_z_;          // [m]
  double max_speed_;                // [m/s]
  double max_height_;               // [m] above the height at the takeoff command
  double halt_tilt_;                // [rad]
  double halt_fall_height_;         // [m] below the height at the takeoff command, with a downward speed
  std::vector<double> geofence_;    // [x_min, x_max, y_min, y_max] in the world frame, empty = none
  double tilt_over_since_;          // [s] -1 = not over
  double takeoff_ground_height_;    // [m] z at the takeoff command
  uint8_t prev_navi_state_;
  ros::Publisher supervisor_pub_;   // std_msgs/String: what tripped
  void supervisorAction(const std::string& reason, bool halt);

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
