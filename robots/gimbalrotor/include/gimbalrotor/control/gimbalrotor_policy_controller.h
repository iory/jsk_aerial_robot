// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

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

  static const int OBS_DIM = 30;
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
  double timeout_;
  double hover_thrust_;   // [N] per rotor, mass * g / 4 of the robot model
  double thrust_max_;     // [N]
  double thrust_scale_;   // the policy's thrust [N] times this goes out (gazebo's spinal makes about twice the
                          // commanded thrust: 0.5 there, 1 on the real machine)
  double gimbal_limit_;   // [rad]
  double landed_height_;  // [m] above the height at arming: LAND_STATE below it is the "landed" phase
  bool active_;
  std::vector<double> command_;       // the policy's last command (normalized, 8)
  double command_stamp_;
  std::vector<double> last_applied_;  // what went out at the previous step, normalized (8)

  void commandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void enableCallback(const std_msgs::Bool::ConstPtr& msg);
  std::vector<float> observation() const;
  bool policyCommandUsable() const;
};
};  // namespace aerial_robot_control
