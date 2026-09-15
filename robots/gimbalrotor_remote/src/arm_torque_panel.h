// -*- mode: c++ -*-

#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>
#include <spinal/ServoTorqueStates.h>

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class QGridLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;

namespace gimbalrotor_remote
{
/*
 * rviz panel to turn the servo torque of the arm joints on and off.
 *
 * - joints : "<robot_ns>/<arm_controller_ns>/joints", servo ids from "<robot_ns>/servo_controller"
 * - state  : "<robot_ns>/servo/torque_states" (spinal::ServoTorqueStates, indexed by servo id)
 * - command: std_srvs/SetBool services "<robot_ns>/<torque_service_ns>/<joint>" and ".../all"
 *            advertised by gimbalrotor/script/arm_torque_server.py
 *
 * Service calls run in a worker thread so that rviz keeps rendering while the server waits for the
 * torque state to follow.
 */
class ArmTorquePanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit ArmTorquePanel(QWidget* parent = nullptr);
  ~ArmTorquePanel() override;

  void onInitialize() override;
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void reload();
  void refreshStates();
  void onServiceDone(bool success, QString message);

private:
  void torqueStatesCallback(const spinal::ServoTorqueStatesConstPtr& msg);
  void requestTorque(const std::string& joint, bool enable);
  void setBusy(bool busy);

  ros::NodeHandle nh_;
  ros::Subscriber torque_states_sub_;

  QLineEdit* robot_ns_edit_;
  QLineEdit* arm_controller_ns_edit_;
  QLineEdit* torque_service_ns_edit_;
  QGridLayout* joints_layout_;
  QLabel* status_label_;
  QTimer* refresh_timer_;
  std::vector<QPushButton*> all_buttons_;
  std::vector<QPushButton*> joint_buttons_;
  std::vector<QLabel*> state_labels_;

  std::vector<std::string> joint_names_;
  std::map<std::string, int> servo_ids_;

  std::mutex state_mutex_;
  std::vector<uint8_t> torque_enable_;
  ros::WallTime last_state_time_;

  std::thread worker_;
  std::atomic<bool> busy_{ false };
};
}  // namespace gimbalrotor_remote
