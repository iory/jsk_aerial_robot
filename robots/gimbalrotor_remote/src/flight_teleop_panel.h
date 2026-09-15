// -*- mode: c++ -*-

#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

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

  ros::NodeHandle nh_;
  ros::Publisher start_pub_, takeoff_pub_, land_pub_, halt_pub_, force_landing_pub_;
  ros::Publisher nav_pub_;
  ros::Subscriber flight_state_sub_, odom_sub_, battery_sub_;

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
  QLabel* status_label_;
  QTimer* timer_;

  std::mutex state_mutex_;
  int flight_state_ = -1;
  ros::WallTime last_flight_state_time_;
  double pos_x_ = 0.0, pos_y_ = 0.0, pos_z_ = 0.0;
  ros::WallTime last_odom_time_;
  double battery_voltage_ = 0.0;
  ros::WallTime last_battery_time_;

  /* axes commanded at the previous update, to send zero once they are released */
  bool xy_active_ = false, z_active_ = false, yaw_active_ = false;
};
}  // namespace gimbalrotor_remote
