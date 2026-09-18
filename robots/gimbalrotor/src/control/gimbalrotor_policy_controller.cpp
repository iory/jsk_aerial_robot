#include <gimbalrotor/control/gimbalrotor_policy_controller.h>
#include <angles/angles.h>

namespace aerial_robot_control
{
GimbalrotorPolicyController::GimbalrotorPolicyController()
  : GimbalrotorController()
  , enabled_(false)
  , timeout_(0.1)
  , fallback_time_(0.3)
  , latched_off_(false)
  , hover_thrust_(0)
  , thrust_max_(30.0)
  , thrust_scale_(1.0)
  , gimbal_limit_(0.785)
  , landed_height_(0.05)
  , vel_lpf_hz_(0.0)
  , integral_limit_(1.0)
  , vel_filtered_(0, 0, 0)
  , pos_error_integral_(0, 0, 0)
  , vel_filter_init_(false)
  , active_(false)
  , command_(ACTION_DIM, 0.0)
  , command_stamp_(-1)
  , last_applied_(ACTION_DIM, 0.0)
{
}

void GimbalrotorPolicyController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                             boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                             boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                             boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                             double ctrl_loop_rate)
{
  GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  ros::NodeHandle policy_nh(nh_, "controller/policy");
  getParam<bool>(policy_nh, "enable", enabled_, false);
  getParam<double>(policy_nh, "timeout", timeout_, 0.1);
  getParam<double>(policy_nh, "fallback_time", fallback_time_, 0.3);
  getParam<double>(policy_nh, "thrust_max", thrust_max_, 30.0);
  getParam<double>(policy_nh, "thrust_scale", thrust_scale_, 1.0);
  getParam<double>(policy_nh, "landed_height", landed_height_, 0.05);
  getParam<double>(policy_nh, "vel_lpf_hz", vel_lpf_hz_, 0.0);
  getParam<double>(policy_nh, "integral_limit", integral_limit_, 1.0);
  double gimbal_limit_default = gimbal_angle_limit_ > 0 ? gimbal_angle_limit_ : 0.785;
  getParam<double>(policy_nh, "gimbal_limit", gimbal_limit_, gimbal_limit_default);
  // the robot model has no mass until its first kinematics update: the hover thrust is taken in controlCore
  hover_thrust_ = 0.0;
  ROS_INFO_STREAM("gimbalrotor policy controller: thrust max " << thrust_max_ << " N, thrust scale " << thrust_scale_
                                                                  << " N, gimbal limit " << gimbal_limit_
                                                                  << " rad, timeout " << timeout_ << " s, velocity low-pass "
                                                                  << vel_lpf_hz_ << " Hz, "
                                                                  << (enabled_ ? "enabled" : "disabled (PID)"));

  observation_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("policy/observation", 1);
  active_pub_ = nh_.advertise<std_msgs::Bool>("policy/active", 1);
  command_sub_ = nh_.subscribe("policy/command", 1, &GimbalrotorPolicyController::commandCallback, this,
                               ros::TransportHints().tcpNoDelay());
  enable_sub_ = nh_.subscribe("policy/enable", 1, &GimbalrotorPolicyController::enableCallback, this);

  std::fill(last_applied_.begin(), last_applied_.end(), 0.0);
  for (int i = 0; i < motor_num_; i++)
    last_applied_.at(i) = -1.0;  // rotors off
}

void GimbalrotorPolicyController::reset()
{
  GimbalrotorController::reset();
  active_ = false;
  vel_filter_init_ = false;
  pos_error_integral_.setValue(0, 0, 0);
  std::fill(last_applied_.begin(), last_applied_.end(), 0.0);
  for (int i = 0; i < motor_num_; i++)
    last_applied_.at(i) = -1.0;
}

void GimbalrotorPolicyController::commandCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (msg->data.size() != (size_t)ACTION_DIM)
  {
    ROS_WARN_THROTTLE(1.0, "policy/command has %zu values, expected %d", msg->data.size(), ACTION_DIM);
    return;
  }
  for (int i = 0; i < ACTION_DIM; i++)
  {
    if (!std::isfinite(msg->data.at(i)))
    {
      ROS_WARN_THROTTLE(1.0, "policy/command is not finite, ignored");
      return;
    }
    command_.at(i) = msg->data.at(i);
  }
  command_stamp_ = ros::Time::now().toSec();
}

void GimbalrotorPolicyController::enableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data != enabled_ || (msg->data && latched_off_))
    ROS_WARN_STREAM("policy " << (msg->data ? "ENABLED" : "DISABLED: PID in control"));
  enabled_ = msg->data;
  if (msg->data)
    latched_off_ = false;  // an explicit enable clears the fallback latch
}

bool GimbalrotorPolicyController::policyCommandUsable() const
{
  if (!enabled_ || latched_off_ || command_stamp_ < 0)
    return false;
  return ros::Time::now().toSec() - command_stamp_ <= fallback_time_;
}

std::vector<float> GimbalrotorPolicyController::observation()
{
  /* the layout of gimbalrotor_mjlab/env_cfg.py (actor group):
     projected gravity (3), body rates (3), position error (3) and velocity error (3) in the heading frame,
     sin / cos of the yaw error (2), gimbal angles (4), the last command (8), the flight phase one-hot (4) */
  std::vector<float> obs;
  obs.reserve(OBS_DIM_INTEGRAL);

  tf::Matrix3x3 rot;
  rot.setRPY(rpy_.x(), rpy_.y(), rpy_.z());
  tf::Vector3 gravity_b = rot.inverse() * tf::Vector3(0, 0, -1);
  obs.push_back(gravity_b.x());
  obs.push_back(gravity_b.y());
  obs.push_back(gravity_b.z());
  obs.push_back(omega_.x());
  obs.push_back(omega_.y());
  obs.push_back(omega_.z());

  tf::Matrix3x3 yaw_rot;
  yaw_rot.setRPY(0, 0, rpy_.z());
  const double dt = ctrl_loop_du_;
  tf::Vector3 vel = vel_;
  if (vel_lpf_hz_ > 0)
  {
    if (!vel_filter_init_)
    {
      vel_filtered_ = vel_;
      vel_filter_init_ = true;
    }
    const double alpha = 1.0 - exp(-2.0 * M_PI * vel_lpf_hz_ * dt);
    vel_filtered_ += alpha * (vel_ - vel_filtered_);
    vel = vel_filtered_;
  }
  tf::Vector3 pos_err = yaw_rot.inverse() * (target_pos_ - pos_);
  tf::Vector3 vel_err = yaw_rot.inverse() * (target_vel_ - vel);
  obs.push_back(pos_err.x());
  obs.push_back(pos_err.y());
  obs.push_back(pos_err.z());
  obs.push_back(vel_err.x());
  obs.push_back(vel_err.y());
  obs.push_back(vel_err.z());
  double yaw_err = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
  obs.push_back(sin(yaw_err));
  obs.push_back(cos(yaw_err));

  /* the integral of the position error (world frame, in the air only), as the training's mission gave it */
  const uint8_t navi = navigator_->getNaviState();
  const bool in_air = navi == aerial_robot_navigation::TAKEOFF_STATE || navi == aerial_robot_navigation::HOVER_STATE ||
                      navi == aerial_robot_navigation::LAND_STATE;
  if (in_air)
    pos_error_integral_ += (target_pos_ - pos_) * dt;
  else
    pos_error_integral_.setValue(0, 0, 0);
  for (int i = 0; i < 3; i++)
    pos_error_integral_[i] = std::max(-integral_limit_, std::min(integral_limit_, (double)pos_error_integral_[i]));
  /* always published (33 values); policy_node.py drops these three for a network that takes 30 */
  tf::Vector3 integral_h = yaw_rot.inverse() * pos_error_integral_;
  obs.push_back(integral_h.x());
  obs.push_back(integral_h.y());
  obs.push_back(integral_h.z());

  const auto& joint_positions = gimbalrotor_robot_model_->getJointPositions();
  const auto& joint_index = gimbalrotor_robot_model_->getJointIndexMap();
  for (int i = 0; i < motor_num_; i++)
  {
    const auto it = joint_index.find("gimbal" + std::to_string(i + 1));
    obs.push_back(it == joint_index.end() ? 0.0 : joint_positions(it->second));
  }

  for (int i = 0; i < ACTION_DIM; i++)
    obs.push_back(last_applied_.at(i));

  /* the flight phase as the mission of the training gave it: before the takeoff, in flight, landing, landed */
  const uint8_t state = navigator_->getNaviState();
  const double height = pos_.z() - navigator_->getInitHeight();
  int phase = 0;
  if (state == aerial_robot_navigation::TAKEOFF_STATE || state == aerial_robot_navigation::HOVER_STATE)
    phase = 1;
  else if (state == aerial_robot_navigation::LAND_STATE)
    phase = height < landed_height_ ? 3 : 2;
  for (int i = 0; i < 4; i++)
    obs.push_back(i == phase ? 1.0 : 0.0);

  return obs;
}

void GimbalrotorPolicyController::controlCore()
{
  /* the PID runs every step (it is the fallback); while the policy flies, its integrators are held so
     that a switch back does not start from wound-up terms */
  if (hover_thrust_ <= 0)
  {
    const double mass = gimbalrotor_robot_model_->getMass();
    if (mass > 0)
    {
      hover_thrust_ = mass * aerial_robot_estimation::G / motor_num_;
      ROS_INFO_STREAM("gimbalrotor policy controller: hover thrust " << hover_thrust_ << " N per rotor (mass " << mass << " kg)");
    }
  }
  if (enabled_ && !latched_off_ && command_stamp_ >= 0 && active_ &&
      ros::Time::now().toSec() - command_stamp_ > fallback_time_)
  {
    latched_off_ = true;
    ROS_ERROR("policy command %.0f ms old: PID in control until policy/enable is published true again",
              (ros::Time::now().toSec() - command_stamp_) * 1000);
  }
  const bool use_policy = policyCommandUsable() && hover_thrust_ > 0;
  if (use_policy && ros::Time::now().toSec() - command_stamp_ > timeout_)
    ROS_WARN_THROTTLE(1.0, "policy command %.0f ms old: holding the last one",
                      (ros::Time::now().toSec() - command_stamp_) * 1000);
  std::vector<double> err_i(pid_controllers_.size());
  for (size_t i = 0; i < pid_controllers_.size(); i++)
    err_i.at(i) = pid_controllers_.at(i).getErrI();

  GimbalrotorController::controlCore();

  if (use_policy)
  {
    for (size_t i = 0; i < pid_controllers_.size(); i++)
      pid_controllers_.at(i).setErrI(err_i.at(i));
    for (int i = 0; i < motor_num_; i++)
    {
      const double a_thrust = std::max(-1.0, std::min(thrust_max_ / hover_thrust_ - 1.0, command_.at(i)));
      const double a_gimbal = std::max(-1.0, std::min(1.0, command_.at(motor_num_ + i)));
      target_full_thrust_.at(i) = thrust_scale_ * hover_thrust_ * (1.0 + a_thrust);
      target_gimbal_angles_.at(i) = gimbal_limit_ * a_gimbal;
      last_applied_.at(i) = a_thrust;
      last_applied_.at(motor_num_ + i) = a_gimbal;
    }
  }
  else if (hover_thrust_ <= 0)
  {
    /* no model mass yet: rotors off in the "last command" the policy is told about */
    std::fill(last_applied_.begin(), last_applied_.end(), 0.0);
    for (int i = 0; i < motor_num_; i++)
      last_applied_.at(i) = -1.0;
  }
  else
  {
    for (int i = 0; i < motor_num_; i++)
    {
      last_applied_.at(i) = std::max(-1.0, std::min(thrust_max_ / hover_thrust_ - 1.0,
                                                    target_full_thrust_.at(i) / (thrust_scale_ * hover_thrust_) - 1.0));
      last_applied_.at(motor_num_ + i) =
          std::max(-1.0, std::min(1.0, target_gimbal_angles_.at(i) / gimbal_limit_));
    }
  }
  active_ = use_policy;

  std_msgs::Float32MultiArray obs_msg;
  obs_msg.data = observation();
  observation_pub_.publish(obs_msg);
  std_msgs::Bool active_msg;
  active_msg.data = active_;
  active_pub_.publish(active_msg);
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorPolicyController, aerial_robot_control::ControlBase);
