// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

namespace aerial_robot_control
{
/*
 * The PID controller of the gimbalrotor with a learned policy in front of the actuators.
 *
 * Every control step (200 Hz) the PID runs as usual and the policy's observation (the state the PID
 * sees, in the layout the policy was trained on: /mnt/workspace/gimbalrotor_mjlab, env_cfg.py) is
 * published on policy/observation. script/policy_node.py answers on policy/command with the 8
 * normalized outputs; when the policy is enabled (policy/enable, std_msgs/Bool, or the param
 * controller/policy/enable) and the last command is fresher than controller/policy/timeout, those
 * replace the PID's thrusts and gimbal angles in four_axes/command and gimbals_ctrl. Anything else
 * (no node, a stale command, a non-finite value, the policy disabled) leaves the PID in control, so
 * the PID can always be brought back by publishing false on policy/enable.
 */
class GimbalrotorPolicyController : public GimbalrotorController
{
public:
  GimbalrotorPolicyController();
  ~GimbalrotorPolicyController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

  static const int OBS_DIM = 33;           // policy_node.py drops the integral (indices 14-16) for a 30-input network
  static const int OBS_DIM_INTEGRAL = 33;
  static const int ACTION_DIM = 8;

protected:
  void controlCore() override;
  void reset() override;

private:
  ros::Publisher observation_pub_;
  ros::Publisher active_pub_;
  ros::Subscriber command_sub_;
  ros::Subscriber enable_sub_;

  bool enabled_;
  double timeout_;        // [s] a command older than this is held as it is (the policy was trained with dead times
                          // up to 0.08 s) ...
  double fallback_time_;  // [s] ... and older than this hands the flight to the PID, latched until policy/enable
                          // is published true again: alternating PID / policy at 200 Hz flipped the robot in gazebo
  bool latched_off_;
  /* supervisor of the policy (controller/policy/supervisor): while the policy flies, an attitude error above
     max_attitude_error, a distance above max_pos_error from the (ramped) target, a speed above max_speed, or a
     gimbal command at its limit, each for longer than trip_time, or a force landing, hands the flight to the
     PID (latched, as the stale-command fallback). The first tier of the flight supervisor of the navigator. */
  bool supervisor_enable_;
  double max_attitude_error_;   // [rad]
  double max_pos_error_;        // [m]
  double max_speed_;            // [m/s]
  double trip_time_;            // [s]
  double attitude_over_since_, pos_over_since_, speed_over_since_, saturated_since_;
  ros::Publisher fallback_pub_;  // std_msgs/String: why the PID took over
  bool supervisorTrips(std::string& reason);
  double hover_thrust_;   // [N] per rotor, mass * g / 4 of the robot model
  double thrust_max_;     // [N]
  double thrust_scale_;   // the policy's thrust [N] times this goes out (gazebo's spinal makes about twice the
                          // commanded thrust: 0.5 there, 1 on the real machine)
  double gimbal_limit_;   // [rad]
  double landed_height_;  // [m] above the height at arming: LAND_STATE below it is the "landed" phase
  double vel_lpf_hz_;     // first-order low-pass on the velocity the policy sees (0 = none); a noisy estimator
                          // (gazebo's mocap mode) made the policy hover with a 0.3 m offset
  double integral_limit_; // [m s]
  /* the navigator's target is ramped before the policy sees it, as the mission of the training gave it: the
     policy loses the position on a yaw step of 45 deg and more (mjlab, 2026-09-18), the PID limits its yaw error
     to 0.4 rad for the same reason */
  double target_pos_rate_;   // [m/s], 0 = no ramp
  double target_yaw_rate_;   // [rad/s], 0 = no ramp
  tf::Vector3 target_pos_ramped_;
  double target_yaw_ramped_;
  bool target_ramp_init_;
  tf::Vector3 vel_filtered_;
  tf::Vector3 pos_error_integral_;
  bool vel_filter_init_;
  std::vector<float> observation();
  bool active_;
  std::vector<double> command_;       // the policy's last command (normalized, 8)
  double command_stamp_;
  std::vector<double> last_applied_;  // what went out at the previous step, normalized (8)

  void commandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void enableCallback(const std_msgs::Bool::ConstPtr& msg);
  bool policyCommandUsable() const;
};
};  // namespace aerial_robot_control
