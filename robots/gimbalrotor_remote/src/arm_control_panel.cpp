// -*- mode: c++ -*-

#include "arm_control_panel.h"

#include <pluginlib/class_list_macros.h>
#include <std_srvs/SetBool.h>
#include <urdf/model.h>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>

namespace gimbalrotor_remote
{
namespace
{
constexpr double STATE_TIMEOUT = 2.0;  // [s] without state messages, the state is shown as unknown
constexpr int REFRESH_INTERVAL_MS = 200;
constexpr double DEFAULT_DURATION = 3.0;  // [s] of the joint motion
constexpr double ACTION_SERVER_TIMEOUT = 2.0;
constexpr double RESULT_MARGIN = 10.0;  // [s] waited after the trajectory duration for the result
constexpr int ANGLE_DECIMALS = 1;  // [deg]
constexpr double SLIDER_TICKS_PER_DEG = 10.0;  // resolution of the sliders, matched to ANGLE_DECIMALS
const char* DEFAULT_ROBOT_NS = "gimbalrotor";
const char* DEFAULT_ARM_CONTROLLER_NS = "arm/arm_controller";
const char* DEFAULT_TORQUE_SERVICE_NS = "arm_torque";
const char* ALL_JOINTS = "all";

enum Column
{
  NAME = 0,
  TORQUE_STATE,
  TORQUE_ON,
  TORQUE_OFF,
  POSITION,
  TARGET_SLIDER,
  TARGET
};

double toDeg(double rad)
{
  return rad * 180.0 / M_PI;
}

double toRad(double deg)
{
  return deg * M_PI / 180.0;
}
}  // namespace

ArmControlPanel::ArmControlPanel(QWidget* parent) : rviz::Panel(parent)
{
  auto layout = new QVBoxLayout;

  auto ns_layout = new QGridLayout;
  robot_ns_edit_ = new QLineEdit(DEFAULT_ROBOT_NS);
  arm_controller_ns_edit_ = new QLineEdit(DEFAULT_ARM_CONTROLLER_NS);
  torque_service_ns_edit_ = new QLineEdit(DEFAULT_TORQUE_SERVICE_NS);
  ns_layout->addWidget(new QLabel("robot ns"), 0, 0);
  ns_layout->addWidget(robot_ns_edit_, 0, 1);
  ns_layout->addWidget(new QLabel("arm controller ns"), 1, 0);
  ns_layout->addWidget(arm_controller_ns_edit_, 1, 1);
  ns_layout->addWidget(new QLabel("torque service ns"), 2, 0);
  ns_layout->addWidget(torque_service_ns_edit_, 2, 1);
  auto reload_button = new QPushButton("Reload joints");
  ns_layout->addWidget(reload_button, 3, 0, 1, 2);
  layout->addLayout(ns_layout);

  joints_layout_ = new QGridLayout;
  layout->addLayout(joints_layout_);

  auto torque_layout = new QHBoxLayout;
  auto all_on = new QPushButton("Torque all ON");
  auto all_off = new QPushButton("Torque all OFF");
  torque_layout->addWidget(all_on);
  torque_layout->addWidget(all_off);
  layout->addLayout(torque_layout);

  auto target_layout = new QHBoxLayout;
  auto copy_button = new QPushButton("Target <- current");
  auto init_pose_button = new QPushButton("Target <- init pose (0 deg)");
  target_layout->addWidget(copy_button);
  target_layout->addWidget(init_pose_button);
  layout->addLayout(target_layout);

  auto motion_layout = new QHBoxLayout;
  duration_spin_ = new QDoubleSpinBox;
  duration_spin_->setRange(0.5, 30.0);
  duration_spin_->setSingleStep(0.5);
  duration_spin_->setValue(DEFAULT_DURATION);
  duration_spin_->setSuffix(" s");
  auto move_button = new QPushButton("Move");
  motion_layout->addWidget(new QLabel("duration"));
  motion_layout->addWidget(duration_spin_);
  motion_layout->addWidget(move_button);
  layout->addLayout(motion_layout);
  move_on_release_check_ = new QCheckBox("move on slider release");
  layout->addWidget(move_on_release_check_);

  fixed_buttons_ = { reload_button, all_on, all_off, copy_button, init_pose_button, move_button };

  status_label_ = new QLabel;
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(reload_button, &QPushButton::clicked, this, &ArmControlPanel::reload);
  connect(all_on, &QPushButton::clicked, this, [this]() { requestTorque(ALL_JOINTS, true); });
  connect(all_off, &QPushButton::clicked, this, [this]() { requestTorque(ALL_JOINTS, false); });
  connect(copy_button, &QPushButton::clicked, this, &ArmControlPanel::copyCurrentToTargets);
  connect(init_pose_button, &QPushButton::clicked, this, &ArmControlPanel::setInitPoseTargets);
  connect(move_button, &QPushButton::clicked, this, &ArmControlPanel::moveToTargets);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &ArmControlPanel::refreshStates);
}

ArmControlPanel::~ArmControlPanel()
{
  if (worker_.joinable())
    worker_.join();
}

void ArmControlPanel::onInitialize()
{
  reload();
  refresh_timer_->start(REFRESH_INTERVAL_MS);
}

void ArmControlPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString value;
  if (config.mapGetString("Robot Namespace", &value))
    robot_ns_edit_->setText(value);
  if (config.mapGetString("Arm Controller Namespace", &value))
    arm_controller_ns_edit_->setText(value);
  if (config.mapGetString("Torque Service Namespace", &value))
    torque_service_ns_edit_->setText(value);
  float duration;
  if (config.mapGetFloat("Duration", &duration))
    duration_spin_->setValue(duration);
  bool move_on_release;
  if (config.mapGetBool("Move On Slider Release", &move_on_release))
    move_on_release_check_->setChecked(move_on_release);
  reload();
}

void ArmControlPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Robot Namespace", robot_ns_edit_->text());
  config.mapSetValue("Arm Controller Namespace", arm_controller_ns_edit_->text());
  config.mapSetValue("Torque Service Namespace", torque_service_ns_edit_->text());
  config.mapSetValue("Duration", duration_spin_->value());
  config.mapSetValue("Move On Slider Release", move_on_release_check_->isChecked());
}

void ArmControlPanel::reload()
{
  if (busy_)
    return;
  const std::string robot_ns = "/" + robot_ns_edit_->text().toStdString();
  const std::string controller_ns = robot_ns + "/" + arm_controller_ns_edit_->text().toStdString();

  /* remove the rows of the previous joints */
  QLayoutItem* item;
  while ((item = joints_layout_->takeAt(0)) != nullptr)
  {
    if (item->widget())
      item->widget()->deleteLater();
    delete item;
  }
  state_labels_.clear();
  position_labels_.clear();
  target_spins_.clear();
  target_sliders_.clear();
  joint_buttons_.clear();
  joint_names_.clear();
  servo_ids_.clear();
  targets_synced_ = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    positions_.clear();
  }

  const std::string joints_param = controller_ns + "/joints";
  if (!nh_.getParam(joints_param, joint_names_) || joint_names_.empty())
  {
    status_label_->setText(QString("rosparam %1 is not found").arg(joints_param.c_str()));
    return;
  }

  XmlRpc::XmlRpcValue servo_controller;
  if (nh_.getParam(robot_ns + "/servo_controller", servo_controller) &&
      servo_controller.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (auto& group : servo_controller)
    {
      if (group.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;
      for (auto& servo : group.second)
      {
        if (servo.first.find("controller") == std::string::npos ||
            servo.second.getType() != XmlRpc::XmlRpcValue::TypeStruct || !servo.second.hasMember("name") ||
            !servo.second.hasMember("id"))
          continue;
        servo_ids_[static_cast<std::string>(servo.second["name"])] = static_cast<int>(servo.second["id"]);
      }
    }
  }

  urdf::Model urdf_model;
  const bool urdf_loaded = urdf_model.initParam(robot_ns + "/robot_description");

  joints_layout_->addWidget(new QLabel("joint"), 0, NAME);
  joints_layout_->addWidget(new QLabel("torque"), 0, TORQUE_STATE);
  joints_layout_->addWidget(new QLabel("current [deg]"), 0, POSITION);
  joints_layout_->addWidget(new QLabel("target [deg]"), 0, TARGET_SLIDER, 1, 2);
  joints_layout_->setColumnStretch(TARGET_SLIDER, 1);
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    const std::string joint = joint_names_[i];
    const int row = i + 1;
    auto state = new QLabel("?");
    state->setAlignment(Qt::AlignCenter);
    state->setMinimumWidth(40);
    auto on = new QPushButton("ON");
    auto off = new QPushButton("OFF");
    auto position = new QLabel("?");
    position->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    position->setMinimumWidth(60);
    auto target = new QDoubleSpinBox;
    target->setDecimals(ANGLE_DECIMALS);
    target->setSingleStep(1.0);
    target->setKeyboardTracking(false);
    auto slider = new QSlider(Qt::Horizontal);
    slider->setMinimumWidth(120);
    double lower = -M_PI, upper = M_PI;
    if (urdf_loaded)
    {
      auto urdf_joint = urdf_model.getJoint(joint);
      if (urdf_joint && urdf_joint->limits && urdf_joint->type != urdf::Joint::CONTINUOUS)
      {
        lower = urdf_joint->limits->lower;
        upper = urdf_joint->limits->upper;
      }
    }
    target->setRange(toDeg(lower), toDeg(upper));
    slider->setRange(std::ceil(toDeg(lower) * SLIDER_TICKS_PER_DEG), std::floor(toDeg(upper) * SLIDER_TICKS_PER_DEG));

    joints_layout_->addWidget(new QLabel(joint.c_str()), row, NAME);
    joints_layout_->addWidget(state, row, TORQUE_STATE);
    joints_layout_->addWidget(on, row, TORQUE_ON);
    joints_layout_->addWidget(off, row, TORQUE_OFF);
    joints_layout_->addWidget(position, row, POSITION);
    joints_layout_->addWidget(slider, row, TARGET_SLIDER);
    joints_layout_->addWidget(target, row, TARGET);
    connect(on, &QPushButton::clicked, this, [this, joint]() { requestTorque(joint, true); });
    connect(off, &QPushButton::clicked, this, [this, joint]() { requestTorque(joint, false); });
    connect(slider, &QSlider::valueChanged, target, [target](int value) {
      const QSignalBlocker blocker(target);
      target->setValue(value / SLIDER_TICKS_PER_DEG);
    });
    connect(target, QOverload<double>::of(&QDoubleSpinBox::valueChanged), slider, [slider](double value) {
      const QSignalBlocker blocker(slider);
      slider->setValue(std::round(value * SLIDER_TICKS_PER_DEG));
    });
    connect(slider, &QSlider::sliderReleased, this, [this]() {
      if (move_on_release_check_->isChecked())
        moveToTargets();
    });
    state_labels_.push_back(state);
    position_labels_.push_back(position);
    target_spins_.push_back(target);
    target_sliders_.push_back(slider);
    joint_buttons_.push_back(on);
    joint_buttons_.push_back(off);
  }

  torque_states_sub_ =
      nh_.subscribe(robot_ns + "/servo/torque_states", 1, &ArmControlPanel::torqueStatesCallback, this);
  joint_states_sub_ = nh_.subscribe(robot_ns + "/joint_states", 1, &ArmControlPanel::jointStatesCallback, this);
  trajectory_client_ = std::make_unique<TrajectoryClient>(nh_, controller_ns + "/follow_joint_trajectory", false);

  status_label_->setText(QString("%1 joints of %2%3")
                             .arg(joint_names_.size())
                             .arg(joints_param.c_str())
                             .arg(urdf_loaded ? "" : " (robot_description not found: targets limited to +-pi)"));
  setBusy(busy_);
}

void ArmControlPanel::torqueStatesCallback(const spinal::ServoTorqueStatesConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  torque_enable_ = msg->torque_enable;
  last_torque_state_time_ = ros::WallTime::now();
}

void ArmControlPanel::jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++)
    positions_[msg->name[i]] = msg->position[i];
  last_joint_state_time_ = ros::WallTime::now();
}

bool ArmControlPanel::currentPositions(std::vector<double>& positions)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (last_joint_state_time_.isZero() || (ros::WallTime::now() - last_joint_state_time_).toSec() > STATE_TIMEOUT)
    return false;
  positions.clear();
  for (const auto& joint : joint_names_)
  {
    auto it = positions_.find(joint);
    if (it == positions_.end())
      return false;
    positions.push_back(it->second);
  }
  return true;
}

void ArmControlPanel::refreshStates()
{
  std::vector<uint8_t> torque_enable;
  bool torque_fresh;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    torque_enable = torque_enable_;
    torque_fresh = !last_torque_state_time_.isZero() &&
                   (ros::WallTime::now() - last_torque_state_time_).toSec() < STATE_TIMEOUT;
  }
  for (size_t i = 0; i < joint_names_.size() && i < state_labels_.size(); i++)
  {
    auto it = servo_ids_.find(joint_names_[i]);
    QLabel* label = state_labels_[i];
    if (!torque_fresh || it == servo_ids_.end() || it->second < 0 ||
        it->second >= static_cast<int>(torque_enable.size()))
    {
      label->setText("?");
      label->setStyleSheet("QLabel { background-color: gray; color: white; }");
    }
    else if (torque_enable[it->second])
    {
      label->setText("ON");
      label->setStyleSheet("QLabel { background-color: green; color: white; }");
    }
    else
    {
      label->setText("OFF");
      label->setStyleSheet("QLabel { background-color: red; color: white; }");
    }
  }

  std::vector<double> positions;
  const bool positions_fresh = currentPositions(positions);
  const double display_resolution = 0.5 * std::pow(10.0, -ANGLE_DECIMALS);
  for (size_t i = 0; i < position_labels_.size(); i++)
  {
    if (!positions_fresh)
    {
      position_labels_[i]->setText("?");
      continue;
    }
    double deg = toDeg(positions[i]);
    if (std::abs(deg) < display_resolution)
      deg = 0.0;  // avoid "-0.0"
    position_labels_[i]->setText(QString::number(deg, 'f', ANGLE_DECIMALS));
  }
  if (positions_fresh && !targets_synced_)
    copyCurrentToTargets();
}

void ArmControlPanel::copyCurrentToTargets()
{
  std::vector<double> positions;
  if (!currentPositions(positions))
  {
    status_label_->setText("joint_states of the arm are not received");
    return;
  }
  for (size_t i = 0; i < target_spins_.size(); i++)
    target_spins_[i]->setValue(toDeg(positions[i]));
  targets_synced_ = true;
}

void ArmControlPanel::setInitPoseTargets()
{
  /* QDoubleSpinBox clamps 0 into the joint limits */
  for (auto spin : target_spins_)
    spin->setValue(0.0);
}

void ArmControlPanel::moveToTargets()
{
  if (busy_ || !trajectory_client_ || target_spins_.empty())
    return;
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = joint_names_;
  trajectory_msgs::JointTrajectoryPoint point;
  for (auto spin : target_spins_)
    point.positions.push_back(toRad(spin->value()));
  const double duration = duration_spin_->value();
  point.time_from_start = ros::Duration(duration);
  goal.trajectory.points.push_back(point);

  startWorker("move", [this, goal, duration]() {
    bool success = false;
    QString message;
    if (!trajectory_client_->waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT)))
      message = "the FollowJointTrajectory action server of the arm controller is not available";
    else
    {
      trajectory_client_->sendGoal(goal);
      if (!trajectory_client_->waitForResult(ros::Duration(duration + RESULT_MARGIN)))
        message = "no result of the trajectory";
      else
      {
        auto result = trajectory_client_->getResult();
        success = result && result->error_code == control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
        message = QString("move %1 (error_code %2) %3")
                      .arg(trajectory_client_->getState().toString().c_str())
                      .arg(result ? result->error_code : 0)
                      .arg(result ? result->error_string.c_str() : "");
      }
    }
    QMetaObject::invokeMethod(this, "onRequestDone", Qt::QueuedConnection, Q_ARG(bool, success),
                              Q_ARG(QString, message), Q_ARG(bool, false));
  });
}

void ArmControlPanel::requestTorque(const std::string& joint, bool enable)
{
  if (busy_)
    return;
  const std::string service =
      "/" + robot_ns_edit_->text().toStdString() + "/" + torque_service_ns_edit_->text().toStdString() + "/" + joint;
  startWorker(QString("torque %1: %2").arg(enable ? "on" : "off").arg(joint.c_str()), [this, service, enable]() {
    std_srvs::SetBool srv;
    srv.request.data = enable;
    bool success = false;
    QString message;
    if (!ros::service::exists(service, false))
      message = QString("service %1 is not available (is arm_torque_server.py running?)").arg(service.c_str());
    else if (!ros::service::call(service, srv))
      message = QString("failed to call %1").arg(service.c_str());
    else
    {
      success = srv.response.success;
      message = QString::fromStdString(srv.response.message);
    }
    QMetaObject::invokeMethod(this, "onRequestDone", Qt::QueuedConnection, Q_ARG(bool, success),
                              Q_ARG(QString, message), Q_ARG(bool, true));
  });
}

void ArmControlPanel::startWorker(const QString& label, std::function<void()> job)
{
  if (worker_.joinable())
    worker_.join();
  setBusy(true);
  status_label_->setStyleSheet("");
  status_label_->setText(label + " ...");
  worker_ = std::thread(job);
}

void ArmControlPanel::onRequestDone(bool success, QString message, bool sync_targets)
{
  status_label_->setText(QString("%1: %2").arg(success ? "done" : "FAILED").arg(message));
  status_label_->setStyleSheet(success ? "" : "QLabel { color: red; }");
  setBusy(false);
  /* the torque server moves the controller target to the measured angles; follow it */
  if (sync_targets)
    copyCurrentToTargets();
}

void ArmControlPanel::setBusy(bool busy)
{
  busy_ = busy;
  for (auto button : fixed_buttons_)
    button->setEnabled(!busy);
  for (auto button : joint_buttons_)
    button->setEnabled(!busy);
  for (auto slider : target_sliders_)
    slider->setEnabled(!busy);
}
}  // namespace gimbalrotor_remote

PLUGINLIB_EXPORT_CLASS(gimbalrotor_remote::ArmControlPanel, rviz::Panel)
