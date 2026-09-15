// -*- mode: c++ -*-

#include <gimbalrotor/arm/arm_hardware_interface.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace gimbalrotor;

bool ArmHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  std::string servo_group;
  robot_hw_nh.param("servo_group", servo_group, std::string("joints"));
  robot_hw_nh.param("joint_state_timeout", joint_state_timeout_, 0.5);
  robot_hw_nh.param("command_resend_interval", command_resend_interval_, 1.0);

  /* joint names: all servos of the servo group in servo_bridge config (e.g. config/grape_with_arm/Servo.yaml) */
  const std::string group_param = "servo_controller/" + servo_group;
  XmlRpc::XmlRpcValue group_params;
  if (!root_nh.getParam(group_param, group_params) || group_params.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM("[arm hw] can not find rosparam " << root_nh.resolveName(group_param));
    return false;
  }
  for (auto& servo_params : group_params)
  {
    if (servo_params.first.find("controller") == std::string::npos)
      continue;
    if (servo_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct || !servo_params.second.hasMember("name") ||
        !servo_params.second.hasMember("id"))
    {
      ROS_ERROR_STREAM("[arm hw] " << group_param << "/" << servo_params.first << " does not have 'name' and 'id'");
      return false;
    }
    servo_id_indices_[static_cast<int>(servo_params.second["id"])] = joint_names_.size();
    joint_names_.push_back(static_cast<std::string>(servo_params.second["name"]));
  }
  if (joint_names_.empty())
  {
    ROS_ERROR_STREAM("[arm hw] no servo is found in " << root_nh.resolveName(group_param));
    return false;
  }

  const size_t n = joint_names_.size();
  position_.assign(n, 0.0);
  velocity_.assign(n, 0.0);  // servo_bridge does not provide velocity
  effort_.assign(n, 0.0);
  command_.assign(n, std::numeric_limits<double>::quiet_NaN());
  received_position_.assign(n, 0.0);
  received_effort_.assign(n, 0.0);
  received_.assign(n, false);
  torque_disabled_.assign(n, false);
  last_published_command_.assign(n, std::numeric_limits<double>::quiet_NaN());

  for (size_t i = 0; i < n; i++)
  {
    joint_indices_[joint_names_[i]] = i;
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &position_[i], &velocity_[i], &effort_[i]);
    joint_state_interface_.registerHandle(state_handle);
    position_joint_interface_.registerHandle(
        hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &command_[i]));
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);

  joint_state_sub_ = root_nh.subscribe("joint_states", 10, &ArmHardwareInterface::jointStateCallback, this,
                                       ros::TransportHints().tcpNoDelay());
  command_pub_ = root_nh.advertise<sensor_msgs::JointState>(servo_group + "_ctrl", 1);
  std::string torque_command_topic;
  robot_hw_nh.param("torque_command_topic", torque_command_topic, std::string("servo/torque_enable"));
  torque_command_sub_ = root_nh.subscribe(torque_command_topic, 10, &ArmHardwareInterface::torqueCommandCallback, this,
                                          ros::TransportHints().tcpNoDelay());

  std::string joints_str;
  for (const auto& name : joint_names_)
    joints_str += " " + name;
  ROS_INFO_STREAM("[arm hw] joints:" << joints_str << ", command topic: " << command_pub_.getTopic());
  return true;
}

void ArmHardwareInterface::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  bool updated = false;
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++)
  {
    auto it = joint_indices_.find(msg->name[i]);
    if (it == joint_indices_.end())
      continue;
    received_position_[it->second] = msg->position[i];
    if (i < msg->effort.size())
      received_effort_[it->second] = msg->effort[i];
    received_[it->second] = true;
    updated = true;
  }
  if (updated)
    last_received_time_ = ros::Time::now();
}

void ArmHardwareInterface::torqueCommandCallback(const spinal::ServoTorqueCmdConstPtr& msg)
{
  if (msg->index.size() != msg->torque_enable.size())
  {
    ROS_ERROR("[arm hw] servo torque command has %zu indices but %zu torque flags", msg->index.size(),
              msg->torque_enable.size());
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->index.size(); i++)
  {
    auto it = servo_id_indices_.find(msg->index[i]);
    if (it == servo_id_indices_.end())
      continue;
    const bool disabled = msg->torque_enable[i] == 0;
    if (torque_disabled_[it->second] != disabled)
      ROS_INFO_STREAM("[arm hw] torque of " << joint_names_[it->second] << (disabled ? " off" : " on"));
    torque_disabled_[it->second] = disabled;
    /* publish the joint in the next write once its torque is back, without waiting for the resend */
    if (!disabled)
      last_published_command_[it->second] = std::numeric_limits<double>::quiet_NaN();
  }
}

bool ArmHardwareInterface::allJointStatesReceived()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (bool received : received_)
  {
    if (!received)
      return false;
  }
  return true;
}

void ArmHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  /* copy without reallocation, since the handles hold pointers to these buffers */
  std::copy(received_position_.begin(), received_position_.end(), position_.begin());
  std::copy(received_effort_.begin(), received_effort_.end(), effort_.begin());

  if (joint_state_timeout_ > 0 && (time - last_received_time_).toSec() > joint_state_timeout_)
  {
    ROS_ERROR_THROTTLE(1.0, "[arm hw] joint_states of the arm are not updated for %.2f [s]",
                       (time - last_received_time_).toSec());
  }
}

void ArmHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  sensor_msgs::JointState msg;
  std::vector<size_t> published;
  bool changed = false;
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    /* NaN: not commanded by any controller yet. torque off: a position command would turn it back on */
    if (!std::isfinite(command_[i]) || torque_disabled_[i])
      continue;
    msg.name.push_back(joint_names_[i]);
    msg.position.push_back(command_[i]);
    published.push_back(i);
    if (command_[i] != last_published_command_[i])
      changed = true;
  }
  if (msg.name.empty())
    return;

  const bool resend =
      command_resend_interval_ > 0 && (time - last_published_time_).toSec() >= command_resend_interval_;
  if (!changed && !resend)
    return;

  msg.header.stamp = time;
  command_pub_.publish(msg);
  for (size_t i : published)
    last_published_command_[i] = command_[i];
  last_published_time_ = time;
}
