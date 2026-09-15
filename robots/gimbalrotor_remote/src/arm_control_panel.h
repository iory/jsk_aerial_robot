// -*- mode: c++ -*-

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#include <spinal/ServoTorqueStates.h>

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class QCheckBox;
class QDoubleSpinBox;
class QGridLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QTimer;

namespace gimbalrotor_remote
{
/*
 * rviz panel for the arm of gimbalrotor: servo torque on/off and joint position targets.
 *
 * - joints : "<robot_ns>/<arm_controller_ns>/joints", servo ids from "<robot_ns>/servo_controller",
 *            limits from "<robot_ns>/robot_description"
 * - state  : "<robot_ns>/servo/torque_states" (spinal::ServoTorqueStates, indexed by servo id),
 *            "<robot_ns>/joint_states"
 * - torque : std_srvs/SetBool services "<robot_ns>/<torque_service_ns>/<joint>" and ".../all"
 *            advertised by gimbalrotor/script/arm_torque_server.py
 * - motion : FollowJointTrajectory action "<robot_ns>/<arm_controller_ns>/follow_joint_trajectory"
 *
 * Angles are shown and entered in degrees; the target of each joint is set with a slider or a spin box.
 * Service calls and action results are waited for in a worker thread so that rviz keeps rendering.
 * The targets are set to the measured angles when the joints are loaded and after every torque
 * command, so that "Move" never sends a stale target. "Target <- init pose" sets all targets to 0 deg
 * (clamped to the joint limits). With "move on slider release", releasing a
 * slider sends the targets at once.
 */
class ArmControlPanel : public rviz::Panel
{
  Q_OBJECT
public:
  explicit ArmControlPanel(QWidget* parent = nullptr);
  ~ArmControlPanel() override;

  void onInitialize() override;
  void load(const rviz::Config& config) override;
  void save(rviz::Config config) const override;

private Q_SLOTS:
  void reload();
  void refreshStates();
  void copyCurrentToTargets();
  void setInitPoseTargets();
  void moveToTargets();
  void onRequestDone(bool success, QString message, bool sync_targets);

private:
  using TrajectoryClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

  void torqueStatesCallback(const spinal::ServoTorqueStatesConstPtr& msg);
  void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  void requestTorque(const std::string& joint, bool enable);
  bool currentPositions(std::vector<double>& positions);
  void startWorker(const QString& label, std::function<void()> job);
  void setBusy(bool busy);

  ros::NodeHandle nh_;
  ros::Subscriber torque_states_sub_;
  ros::Subscriber joint_states_sub_;
  std::unique_ptr<TrajectoryClient> trajectory_client_;

  QLineEdit* robot_ns_edit_;
  QLineEdit* arm_controller_ns_edit_;
  QLineEdit* torque_service_ns_edit_;
  QGridLayout* joints_layout_;
  QDoubleSpinBox* duration_spin_;
  QCheckBox* move_on_release_check_;
  QLabel* status_label_;
  QTimer* refresh_timer_;
  std::vector<QPushButton*> fixed_buttons_;
  std::vector<QPushButton*> joint_buttons_;
  std::vector<QLabel*> state_labels_;
  std::vector<QLabel*> position_labels_;
  std::vector<QDoubleSpinBox*> target_spins_;
  std::vector<QSlider*> target_sliders_;

  std::vector<std::string> joint_names_;
  std::map<std::string, int> servo_ids_;
  bool targets_synced_ = false;

  std::mutex state_mutex_;
  std::vector<uint8_t> torque_enable_;
  ros::WallTime last_torque_state_time_;
  std::map<std::string, double> positions_;
  ros::WallTime last_joint_state_time_;

  std::thread worker_;
  std::atomic<bool> busy_{ false };
};
}  // namespace gimbalrotor_remote
