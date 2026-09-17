// -*- mode: c++ -*-

#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#include <spinal/ServoTorqueStates.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace gimbalrotor_remote
{
/*
 * rviz panel for the basic teleoperation of an aerial robot, equivalent to aerial_robot_base/keyboard_command.py.
 *
 * - commands : std_msgs/Empty "<robot_ns>/teleop_command/{start,takeoff,land,halt,force_landing}"
 * - motion   : aerial_robot_msgs/FlightNav "<robot_ns>/uav/nav" in VEL_MODE
 * - state    : "<robot_ns>/flight_state", "<robot_ns>/uav/cog/odom", "<robot_ns>/battery_voltage_status"
 *
 * The navigator resets a velocity target to zero "teleop_reset_duration" (0.5 s) after the last command, so a
 * direction button publishes its velocity periodically while it is held down and zero once it is released.
 * The navigator accepts uav/nav only in HOVER_STATE, so the direction buttons are enabled only then.
 * Arming and takeoff are enabled only while "enable arming / takeoff" is checked; it is never restored from the
 * rviz config. Land, halt and force landing are always enabled.
 *
 * gimbals (only in ARM_OFF, the motors stopped):
 * - torque on / off : std_srvs/SetBool "<robot_ns>/gimbals/torque_enable" of servo_bridge
 * - to 0 deg        : torque on, then sensor_msgs/JointState "<robot_ns>/gimbals_ctrl" with every gimbal at 0
 * - set current as 0: torque off, then spinal/SetBoardConfig "<robot_ns>/set_board_config" SET_SERVO_HOMING_OFFSET
 *                     with the raw value of 0 rad (zero_point_offset of the servo config), which makes the servo
 *                     rewrite its homing offset so that the present pose reads 0 rad (kept in the servo)
 * The gimbal ids, names and zero points come from "<robot_ns>/servo_controller/gimbals" (the Servo.yaml of the
 * robot); the angles and torque flags shown come from "<robot_ns>/joint_states" and "<robot_ns>/servo/torque_states".
 */
class FlightTeleopPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit FlightTeleopPanel(QWidget* parent = nullptr);

  void onInitialize() override;
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void reconnect();
  void update();
  void stop();

private:
  QPushButton* addDirectionButton(const QString& label);
  void publishEmpty(ros::Publisher& pub, const QString& name);
  void flightStateCallback(const std_msgs::UInt8ConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void batteryCallback(const std_msgs::Float32ConstPtr& msg);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void torqueStateCallback(const spinal::ServoTorqueStatesConstPtr& msg);
  bool loadGimbalConfig(const std::string& robot_ns);
  std::vector<int> selectedGimbals() const;
  bool setGimbalTorque(bool enable);
  void sendGimbalZero();
  void setGimbalZeroHere();
  void setStatus(const QString& text, bool error);

  struct GimbalServo
  {
    std::string name;
    int id;
    int zero_point_offset;  // raw value of 0 rad
  };

  ros::NodeHandle nh_;
  ros::Publisher start_pub_, takeoff_pub_, land_pub_, halt_pub_, force_landing_pub_;
  ros::Publisher nav_pub_;
  ros::Subscriber flight_state_sub_, odom_sub_, battery_sub_;
  ros::Publisher gimbal_ctrl_pub_;
  ros::Subscriber joint_state_sub_, torque_state_sub_;
  std::string robot_ns_;
  std::vector<GimbalServo> gimbals_;

  QLineEdit* robot_ns_edit_;
  QLabel* state_label_;
  QLabel* position_label_;
  QLabel* battery_label_;
  QCheckBox* enable_flight_check_;
  QPushButton* start_button_;
  QPushButton* takeoff_button_;
  QComboBox* frame_combo_;
  QDoubleSpinBox* xy_vel_spin_;
  QDoubleSpinBox* z_vel_spin_;
  QDoubleSpinBox* yaw_vel_spin_;
  QPushButton* forward_button_;
  QPushButton* backward_button_;
  QPushButton* left_button_;
  QPushButton* right_button_;
  QPushButton* up_button_;
  QPushButton* down_button_;
  QPushButton* yaw_left_button_;
  QPushButton* yaw_right_button_;
  QPushButton* stop_button_;
  std::vector<QPushButton*> direction_buttons_;
  QLabel* gimbal_label_;
  QComboBox* gimbal_combo_;
  QPushButton* gimbal_on_button_;
  QPushButton* gimbal_off_button_;
  QPushButton* gimbal_zero_button_;
  QPushButton* gimbal_calib_button_;
  QLabel* status_label_;
  QTimer* timer_;

  std::mutex state_mutex_;
  int flight_state_ = -1;
  ros::WallTime last_flight_state_time_;
  double pos_x_ = 0.0, pos_y_ = 0.0, pos_z_ = 0.0;
  ros::WallTime last_odom_time_;
  double battery_voltage_ = 0.0;
  ros::WallTime last_battery_time_;
  std::map<std::string, double> joint_angles_;
  ros::WallTime last_joint_state_time_;
  std::vector<uint8_t> torque_states_;
  ros::WallTime last_torque_state_time_;

  /* axes commanded at the previous update, to send zero once they are released */
  bool xy_active_ = false, z_active_ = false, yaw_active_ = false;
};
}  // namespace gimbalrotor_remote
