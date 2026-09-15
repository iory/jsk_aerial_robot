// -*- mode: c++ -*-

#include "arm_torque_panel.h"

#include <pluginlib/class_list_macros.h>
#include <std_srvs/SetBool.h>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace gimbalrotor_remote
{
namespace
{
constexpr double STATE_TIMEOUT = 2.0;  // [s] without servo/torque_states, the state is shown as unknown
constexpr int REFRESH_INTERVAL_MS = 200;
const char* DEFAULT_ROBOT_NS = "gimbalrotor";
const char* DEFAULT_ARM_CONTROLLER_NS = "arm/arm_controller";
const char* DEFAULT_TORQUE_SERVICE_NS = "arm_torque";
const char* ALL_JOINTS = "all";
}  // namespace

ArmTorquePanel::ArmTorquePanel(QWidget* parent) : rviz::Panel(parent)
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

  auto all_layout = new QHBoxLayout;
  auto all_on = new QPushButton("All ON");
  auto all_off = new QPushButton("All OFF");
  all_layout->addWidget(all_on);
  all_layout->addWidget(all_off);
  layout->addLayout(all_layout);
  all_buttons_ = { all_on, all_off };

  status_label_ = new QLabel;
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(reload_button, &QPushButton::clicked, this, &ArmTorquePanel::reload);
  connect(all_on, &QPushButton::clicked, this, [this]() { requestTorque(ALL_JOINTS, true); });
  connect(all_off, &QPushButton::clicked, this, [this]() { requestTorque(ALL_JOINTS, false); });

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &ArmTorquePanel::refreshStates);
}

ArmTorquePanel::~ArmTorquePanel()
{
  if (worker_.joinable())
    worker_.join();
}

void ArmTorquePanel::onInitialize()
{
  reload();
  refresh_timer_->start(REFRESH_INTERVAL_MS);
}

void ArmTorquePanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString value;
  if (config.mapGetString("Robot Namespace", &value))
    robot_ns_edit_->setText(value);
  if (config.mapGetString("Arm Controller Namespace", &value))
    arm_controller_ns_edit_->setText(value);
  if (config.mapGetString("Torque Service Namespace", &value))
    torque_service_ns_edit_->setText(value);
  reload();
}

void ArmTorquePanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Robot Namespace", robot_ns_edit_->text());
  config.mapSetValue("Arm Controller Namespace", arm_controller_ns_edit_->text());
  config.mapSetValue("Torque Service Namespace", torque_service_ns_edit_->text());
}

void ArmTorquePanel::reload()
{
  const std::string robot_ns = "/" + robot_ns_edit_->text().toStdString();

  /* remove the rows of the previous joints */
  QLayoutItem* item;
  while ((item = joints_layout_->takeAt(0)) != nullptr)
  {
    if (item->widget())
      item->widget()->deleteLater();
    delete item;
  }
  state_labels_.clear();
  joint_buttons_.clear();
  joint_names_.clear();
  servo_ids_.clear();

  const std::string joints_param = robot_ns + "/" + arm_controller_ns_edit_->text().toStdString() + "/joints";
  if (!nh_.getParam(joints_param, joint_names_) || joint_names_.empty())
  {
    status_label_->setText(QString("rosparam %1 is not found").arg(joints_param.c_str()));
    return;
  }

  XmlRpc::XmlRpcValue servo_controller;
  const std::string servo_param = robot_ns + "/servo_controller";
  if (nh_.getParam(servo_param, servo_controller) && servo_controller.getType() == XmlRpc::XmlRpcValue::TypeStruct)
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

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    const std::string joint = joint_names_[i];
    auto state = new QLabel("?");
    state->setAlignment(Qt::AlignCenter);
    state->setMinimumWidth(40);
    auto on = new QPushButton("ON");
    auto off = new QPushButton("OFF");
    joints_layout_->addWidget(new QLabel(joint.c_str()), i, 0);
    joints_layout_->addWidget(state, i, 1);
    joints_layout_->addWidget(on, i, 2);
    joints_layout_->addWidget(off, i, 3);
    connect(on, &QPushButton::clicked, this, [this, joint]() { requestTorque(joint, true); });
    connect(off, &QPushButton::clicked, this, [this, joint]() { requestTorque(joint, false); });
    state_labels_.push_back(state);
    joint_buttons_.push_back(on);
    joint_buttons_.push_back(off);
  }

  torque_states_sub_ =
      nh_.subscribe(robot_ns + "/servo/torque_states", 1, &ArmTorquePanel::torqueStatesCallback, this);
  status_label_->setText(QString("%1 joints of %2").arg(joint_names_.size()).arg(joints_param.c_str()));
  setBusy(busy_);
}

void ArmTorquePanel::torqueStatesCallback(const spinal::ServoTorqueStatesConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  torque_enable_ = msg->torque_enable;
  last_state_time_ = ros::WallTime::now();
}

void ArmTorquePanel::refreshStates()
{
  std::vector<uint8_t> torque_enable;
  bool fresh;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    torque_enable = torque_enable_;
    fresh = !last_state_time_.isZero() && (ros::WallTime::now() - last_state_time_).toSec() < STATE_TIMEOUT;
  }
  for (size_t i = 0; i < joint_names_.size() && i < state_labels_.size(); i++)
  {
    auto it = servo_ids_.find(joint_names_[i]);
    QLabel* label = state_labels_[i];
    if (!fresh || it == servo_ids_.end() || it->second < 0 || it->second >= static_cast<int>(torque_enable.size()))
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
}

void ArmTorquePanel::requestTorque(const std::string& joint, bool enable)
{
  if (busy_)
    return;
  const std::string service =
      "/" + robot_ns_edit_->text().toStdString() + "/" + torque_service_ns_edit_->text().toStdString() + "/" + joint;
  if (worker_.joinable())
    worker_.join();
  setBusy(true);
  status_label_->setText(QString("torque %1: %2 ...").arg(enable ? "on" : "off").arg(joint.c_str()));
  worker_ = std::thread([this, service, enable]() {
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
    QMetaObject::invokeMethod(this, "onServiceDone", Qt::QueuedConnection, Q_ARG(bool, success),
                              Q_ARG(QString, message));
  });
}

void ArmTorquePanel::onServiceDone(bool success, QString message)
{
  status_label_->setText(QString("%1: %2").arg(success ? "done" : "FAILED").arg(message));
  status_label_->setStyleSheet(success ? "" : "QLabel { color: red; }");
  setBusy(false);
}

void ArmTorquePanel::setBusy(bool busy)
{
  busy_ = busy;
  for (auto button : all_buttons_)
    button->setEnabled(!busy);
  for (auto button : joint_buttons_)
    button->setEnabled(!busy);
}
}  // namespace gimbalrotor_remote

PLUGINLIB_EXPORT_CLASS(gimbalrotor_remote::ArmTorquePanel, rviz::Panel)
