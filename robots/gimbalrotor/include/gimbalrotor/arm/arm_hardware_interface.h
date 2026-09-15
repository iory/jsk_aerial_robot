// -*- mode: c++ -*-

#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace gimbalrotor
{
/*
 * ros_control hardware interface for the arm servos handled by servo_bridge.
 *
 * - read : joint positions from "joint_states" (published by servo_bridge or gazebo)
 * - write: target joint positions to "<servo_group>_ctrl" (subscribed by servo_bridge)
 *
 * The command is published only when it changes or when "command_resend_interval" has passed,
 * so that the holding command does not follow the (sagging) measured position.
 */
class ArmHardwareInterface : public hardware_interface::RobotHW
{
public:
  ArmHardwareInterface() = default;
  ~ArmHardwareInterface() override = default;

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  /* true after the positions of all joints are received at least once */
  bool allJointStatesReceived();
  const std::vector<std::string>& getJointNames() const
  {
    return joint_names_;
  }

private:
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  ros::Subscriber joint_state_sub_;
  ros::Publisher command_pub_;

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_indices_;

  /* data exposed to controllers (accessed only in the control loop) */
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> command_;

  /* latest data from the joint_states callback */
  std::mutex state_mutex_;
  std::vector<double> received_position_;
  std::vector<double> received_effort_;
  std::vector<bool> received_;
  ros::Time last_received_time_;

  std::vector<double> last_published_command_;
  ros::Time last_published_time_;

  /* rosparam */
  double joint_state_timeout_;
  double command_resend_interval_;
};
}  // namespace gimbalrotor
