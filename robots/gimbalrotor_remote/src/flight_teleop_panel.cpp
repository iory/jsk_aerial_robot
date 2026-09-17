// -*- mode: c++ -*-

#include "flight_teleop_panel.h"

#include <aerial_robot_msgs/FlightNav.h>
#include <pluginlib/class_list_macros.h>
#include <spinal/SetBoardConfig.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace gimbalrotor_remote
{
namespace
{
constexpr double STATE_TIMEOUT = 2.0;  // [s] without state messages, the state is shown as unknown
constexpr int UPDATE_INTERVAL_MS = 100;  // shorter than teleop_reset_duration of the navigator (0.5 s)
constexpr double DEFAULT_XY_VEL = 0.2;   // [m/s], same as keyboard_command.py
constexpr double DEFAULT_Z_VEL = 0.2;    // [m/s]
constexpr double DEFAULT_YAW_VEL = 10.0;  // [deg/s]
const char* DEFAULT_ROBOT_NS = "gimbalrotor";

constexpr double SERVICE_TIMEOUT = 2.0;  // [s] to wait for a service to appear
/* spinal takes a homing offset only with the torque off: wait for the torque off command to reach it */
constexpr double TORQUE_OFF_WAIT = 0.5;  // [s]

/* aerial_robot_navigation::BaseNavigator::flight_state and the special states of flight_state */
constexpr int ARM_OFF_STATE = 0;
constexpr int HOVER_STATE = 5;
QString flightStateName(int state)
{
  switch (state)
  {
    case 0:
      return "ARM_OFF";
    case 1:
      return "START";
    case 2:
      return "ARM_ON";
    case 3:
      return "TAKEOFF";
    case 4:
      return "LAND";
    case HOVER_STATE:
      return "HOVER";
    case 6:
      return "STOP";
    case 0x10:
      return "LOW_BATTERY";
    case 0x11:
      return "FORCE_LANDING";
    default:
      return QString("UNKNOWN(%1)").arg(state);
  }
}

QString flightStateColor(int state)
{
  switch (state)
  {
    case 0:
      return "gray";
    case HOVER_STATE:
      return "green";
    case 0x10:
    case 0x11:
      return "red";
    default:
      return "darkorange";
  }
}
}  // namespace

FlightTeleopPanel::FlightTeleopPanel(QWidget* parent) : rviz::Panel(parent)
{
  auto layout = new QVBoxLayout;

  auto ns_layout = new QHBoxLayout;
  robot_ns_edit_ = new QLineEdit(DEFAULT_ROBOT_NS);
  auto reconnect_button = new QPushButton("Reconnect");
  ns_layout->addWidget(new QLabel("robot ns"));
  ns_layout->addWidget(robot_ns_edit_);
  ns_layout->addWidget(reconnect_button);
  layout->addLayout(ns_layout);

  auto state_layout = new QHBoxLayout;
  state_label_ = new QLabel("?");
  state_label_->setAlignment(Qt::AlignCenter);
  state_label_->setMinimumWidth(110);
  position_label_ = new QLabel("pos: ?");
  battery_label_ = new QLabel("battery: ?");
  state_layout->addWidget(state_label_);
  state_layout->addWidget(position_label_, 1);
  state_layout->addWidget(battery_label_);
  layout->addLayout(state_layout);

  auto flight_layout = new QGridLayout;
  enable_flight_check_ = new QCheckBox("enable arming / takeoff");
  start_button_ = new QPushButton("Arm (start)");
  takeoff_button_ = new QPushButton("Takeoff");
  auto land_button = new QPushButton("Land");
  auto force_landing_button = new QPushButton("Force landing");
  auto halt_button = new QPushButton("HALT (motor stop)");
  halt_button->setStyleSheet("QPushButton { background-color: #c62828; color: white; font-weight: bold; }");
  flight_layout->addWidget(enable_flight_check_, 0, 0, 1, 3);
  flight_layout->addWidget(start_button_, 1, 0);
  flight_layout->addWidget(takeoff_button_, 1, 1);
  flight_layout->addWidget(land_button, 1, 2);
  flight_layout->addWidget(force_landing_button, 2, 0);
  flight_layout->addWidget(halt_button, 2, 1, 1, 2);
  layout->addLayout(flight_layout);

  auto vel_layout = new QGridLayout;
  frame_combo_ = new QComboBox;
  frame_combo_->addItem("world frame", aerial_robot_msgs::FlightNav::WORLD_FRAME);
  frame_combo_->addItem("body frame", aerial_robot_msgs::FlightNav::LOCAL_FRAME);
  xy_vel_spin_ = new QDoubleSpinBox;
  xy_vel_spin_->setRange(0.01, 1.0);
  xy_vel_spin_->setSingleStep(0.05);
  xy_vel_spin_->setValue(DEFAULT_XY_VEL);
  xy_vel_spin_->setSuffix(" m/s");
  z_vel_spin_ = new QDoubleSpinBox;
  z_vel_spin_->setRange(0.01, 1.0);
  z_vel_spin_->setSingleStep(0.05);
  z_vel_spin_->setValue(DEFAULT_Z_VEL);
  z_vel_spin_->setSuffix(" m/s");
  yaw_vel_spin_ = new QDoubleSpinBox;
  yaw_vel_spin_->setDecimals(1);
  yaw_vel_spin_->setRange(1.0, 90.0);
  yaw_vel_spin_->setSingleStep(5.0);
  yaw_vel_spin_->setValue(DEFAULT_YAW_VEL);
  yaw_vel_spin_->setSuffix(" deg/s");
  vel_layout->addWidget(frame_combo_, 0, 0, 1, 2);
  vel_layout->addWidget(new QLabel("xy"), 1, 0);
  vel_layout->addWidget(xy_vel_spin_, 1, 1);
  vel_layout->addWidget(new QLabel("z"), 2, 0);
  vel_layout->addWidget(z_vel_spin_, 2, 1);
  vel_layout->addWidget(new QLabel("yaw"), 3, 0);
  vel_layout->addWidget(yaw_vel_spin_, 3, 1);

  /* press and hold */
  auto pad_layout = new QGridLayout;
  yaw_left_button_ = addDirectionButton("↺ yaw");
  forward_button_ = addDirectionButton("↑ +x");
  yaw_right_button_ = addDirectionButton("yaw ↻");
  left_button_ = addDirectionButton("← +y");
  stop_button_ = new QPushButton("■ stop");
  right_button_ = addDirectionButton("-y →");
  backward_button_ = addDirectionButton("↓ -x");
  up_button_ = addDirectionButton("▲ up");
  down_button_ = addDirectionButton("▼ down");
  pad_layout->addWidget(yaw_left_button_, 0, 0);
  pad_layout->addWidget(forward_button_, 0, 1);
  pad_layout->addWidget(yaw_right_button_, 0, 2);
  pad_layout->addWidget(left_button_, 1, 0);
  pad_layout->addWidget(stop_button_, 1, 1);
  pad_layout->addWidget(right_button_, 1, 2);
  pad_layout->addWidget(backward_button_, 2, 1);
  pad_layout->addWidget(up_button_, 0, 3);
  pad_layout->addWidget(down_button_, 2, 3);
  pad_layout->setColumnMinimumWidth(3, 70);

  auto motion_layout = new QHBoxLayout;
  motion_layout->addLayout(pad_layout, 1);
  motion_layout->addLayout(vel_layout);
  layout->addLayout(motion_layout);

  auto gimbal_box = new QGroupBox("gimbals (motors stopped only)");
  auto gimbal_layout = new QGridLayout;
  gimbal_label_ = new QLabel("?");
  gimbal_combo_ = new QComboBox;
  gimbal_on_button_ = new QPushButton("Torque ON");
  gimbal_off_button_ = new QPushButton("Torque OFF");
  gimbal_zero_button_ = new QPushButton("Torque ON + 0 deg");
  gimbal_calib_button_ = new QPushButton("Set current as 0 deg");
  gimbal_calib_button_->setToolTip("turns the torque off and rewrites the homing offset of the servo so that the "
                                   "present pose reads 0 deg (kept in the servo)");
  gimbal_layout->addWidget(gimbal_label_, 0, 0, 1, 4);
  gimbal_layout->addWidget(gimbal_on_button_, 1, 0);
  gimbal_layout->addWidget(gimbal_off_button_, 1, 1);
  gimbal_layout->addWidget(gimbal_zero_button_, 1, 2, 1, 2);
  gimbal_layout->addWidget(new QLabel("target"), 2, 0);
  gimbal_layout->addWidget(gimbal_combo_, 2, 1);
  gimbal_layout->addWidget(gimbal_calib_button_, 2, 2, 1, 2);
  gimbal_box->setLayout(gimbal_layout);
  layout->addWidget(gimbal_box);

  status_label_ = new QLabel("direction buttons move the robot while held down (HOVER state only)");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(reconnect_button, &QPushButton::clicked, this, &FlightTeleopPanel::reconnect);
  connect(start_button_, &QPushButton::clicked, this, [this]() { publishEmpty(start_pub_, "arm (start)"); });
  connect(takeoff_button_, &QPushButton::clicked, this, [this]() { publishEmpty(takeoff_pub_, "takeoff"); });
  connect(land_button, &QPushButton::clicked, this, [this]() { publishEmpty(land_pub_, "land"); });
  connect(halt_button, &QPushButton::clicked, this, [this]() { publishEmpty(halt_pub_, "halt"); });
  connect(force_landing_button, &QPushButton::clicked, this,
          [this]() { publishEmpty(force_landing_pub_, "force landing"); });
  connect(stop_button_, &QPushButton::clicked, this, &FlightTeleopPanel::stop);
  connect(gimbal_on_button_, &QPushButton::clicked, this, [this]() {
    if (setGimbalTorque(true))
      setStatus("gimbal torque on", false);
  });
  connect(gimbal_off_button_, &QPushButton::clicked, this, [this]() {
    if (setGimbalTorque(false))
      setStatus("gimbal torque off", false);
  });
  connect(gimbal_zero_button_, &QPushButton::clicked, this, &FlightTeleopPanel::sendGimbalZero);
  connect(gimbal_calib_button_, &QPushButton::clicked, this, &FlightTeleopPanel::setGimbalZeroHere);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &FlightTeleopPanel::update);
}

QPushButton* FlightTeleopPanel::addDirectionButton(const QString& label)
{
  auto button = new QPushButton(label);
  direction_buttons_.push_back(button);
  return button;
}

void FlightTeleopPanel::onInitialize()
{
  reconnect();
  timer_->start(UPDATE_INTERVAL_MS);
}

void FlightTeleopPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString value;
  if (config.mapGetString("Robot Namespace", &value))
    robot_ns_edit_->setText(value);
  int frame;
  if (config.mapGetInt("Frame", &frame))
    frame_combo_->setCurrentIndex(frame_combo_->findData(frame));
  float vel;
  if (config.mapGetFloat("XY Velocity", &vel))
    xy_vel_spin_->setValue(vel);
  if (config.mapGetFloat("Z Velocity", &vel))
    z_vel_spin_->setValue(vel);
  if (config.mapGetFloat("Yaw Velocity", &vel))
    yaw_vel_spin_->setValue(vel);
  reconnect();
}

void FlightTeleopPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("Robot Namespace", robot_ns_edit_->text());
  config.mapSetValue("Frame", frame_combo_->currentData().toInt());
  config.mapSetValue("XY Velocity", xy_vel_spin_->value());
  config.mapSetValue("Z Velocity", z_vel_spin_->value());
  config.mapSetValue("Yaw Velocity", yaw_vel_spin_->value());
}

void FlightTeleopPanel::reconnect()
{
  const std::string robot_ns = "/" + robot_ns_edit_->text().toStdString();
  const std::string teleop_ns = robot_ns + "/teleop_command";
  start_pub_ = nh_.advertise<std_msgs::Empty>(teleop_ns + "/start", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>(teleop_ns + "/takeoff", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>(teleop_ns + "/land", 1);
  halt_pub_ = nh_.advertise<std_msgs::Empty>(teleop_ns + "/halt", 1);
  force_landing_pub_ = nh_.advertise<std_msgs::Empty>(teleop_ns + "/force_landing", 1);
  nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>(robot_ns + "/uav/nav", 1);
  flight_state_sub_ = nh_.subscribe(robot_ns + "/flight_state", 1, &FlightTeleopPanel::flightStateCallback, this);
  odom_sub_ = nh_.subscribe(robot_ns + "/uav/cog/odom", 1, &FlightTeleopPanel::odomCallback, this);
  battery_sub_ = nh_.subscribe(robot_ns + "/battery_voltage_status", 1, &FlightTeleopPanel::batteryCallback, this);
  gimbal_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>(robot_ns + "/gimbals_ctrl", 1);
  joint_state_sub_ = nh_.subscribe(robot_ns + "/joint_states", 1, &FlightTeleopPanel::jointStateCallback, this);
  torque_state_sub_ =
      nh_.subscribe(robot_ns + "/servo/torque_states", 1, &FlightTeleopPanel::torqueStateCallback, this);
  robot_ns_ = robot_ns;
  loadGimbalConfig(robot_ns);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_flight_state_time_ = ros::WallTime();
    last_odom_time_ = ros::WallTime();
    last_battery_time_ = ros::WallTime();
    last_joint_state_time_ = ros::WallTime();
    last_torque_state_time_ = ros::WallTime();
  }
  xy_active_ = z_active_ = yaw_active_ = false;
}

void FlightTeleopPanel::publishEmpty(ros::Publisher& pub, const QString& name)
{
  if (pub.getNumSubscribers() == 0)
  {
    status_label_->setStyleSheet("QLabel { color: red; }");
    status_label_->setText(QString("%1: nobody subscribes %2").arg(name).arg(pub.getTopic().c_str()));
    return;
  }
  pub.publish(std_msgs::Empty());
  status_label_->setStyleSheet("");
  status_label_->setText(QString("sent %1").arg(name));
}

void FlightTeleopPanel::flightStateCallback(const std_msgs::UInt8ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  flight_state_ = msg->data;
  last_flight_state_time_ = ros::WallTime::now();
}

void FlightTeleopPanel::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  pos_x_ = msg->pose.pose.position.x;
  pos_y_ = msg->pose.pose.position.y;
  pos_z_ = msg->pose.pose.position.z;
  last_odom_time_ = ros::WallTime::now();
}

void FlightTeleopPanel::batteryCallback(const std_msgs::Float32ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  battery_voltage_ = msg->data;
  last_battery_time_ = ros::WallTime::now();
}

void FlightTeleopPanel::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++)
    joint_angles_[msg->name[i]] = msg->position[i];
  last_joint_state_time_ = ros::WallTime::now();
}

void FlightTeleopPanel::torqueStateCallback(const spinal::ServoTorqueStatesConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  torque_states_ = msg->torque_enable;
  last_torque_state_time_ = ros::WallTime::now();
}

bool FlightTeleopPanel::loadGimbalConfig(const std::string& robot_ns)
{
  gimbals_.clear();
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam(robot_ns + "/servo_controller/gimbals", config) ||
      config.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    gimbal_combo_->clear();
    return false;
  }
  auto to_int = [](XmlRpc::XmlRpcValue& v) {
    return v.getType() == XmlRpc::XmlRpcValue::TypeDouble ? static_cast<int>(static_cast<double>(v)) :
                                                             static_cast<int>(v);
  };
  const int group_zero = config.hasMember("zero_point_offset") ? to_int(config["zero_point_offset"]) : 0;
  for (auto& item : config)
  {
    if (item.first.find("controller") != 0 || item.second.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !item.second.hasMember("id") || !item.second.hasMember("name"))
      continue;
    GimbalServo servo;
    servo.name = static_cast<std::string>(item.second["name"]);
    servo.id = to_int(item.second["id"]);
    servo.zero_point_offset =
        item.second.hasMember("zero_point_offset") ? to_int(item.second["zero_point_offset"]) : group_zero;
    gimbals_.push_back(servo);
  }
  std::sort(gimbals_.begin(), gimbals_.end(),
            [](const GimbalServo& a, const GimbalServo& b) { return a.name < b.name; });

  const QString current = gimbal_combo_->currentText();
  gimbal_combo_->clear();
  gimbal_combo_->addItem("all", -1);
  for (size_t i = 0; i < gimbals_.size(); i++)
    gimbal_combo_->addItem(QString::fromStdString(gimbals_[i].name), static_cast<int>(i));
  if (gimbal_combo_->findText(current) >= 0)
    gimbal_combo_->setCurrentIndex(gimbal_combo_->findText(current));
  return !gimbals_.empty();
}

std::vector<int> FlightTeleopPanel::selectedGimbals() const
{
  std::vector<int> indices;
  const int selected = gimbal_combo_->currentData().toInt();
  for (size_t i = 0; i < gimbals_.size(); i++)
    if (selected < 0 || selected == static_cast<int>(i))
      indices.push_back(i);
  return indices;
}

void FlightTeleopPanel::setStatus(const QString& text, bool error)
{
  status_label_->setStyleSheet(error ? "QLabel { color: red; }" : "");
  status_label_->setText(text);
}

bool FlightTeleopPanel::setGimbalTorque(bool enable)
{
  const std::string service = robot_ns_ + "/gimbals/torque_enable";
  std_srvs::SetBool srv;
  srv.request.data = enable;
  if (!ros::service::waitForService(service, ros::Duration(SERVICE_TIMEOUT)))
  {
    setStatus(QString("no service %1").arg(service.c_str()), true);
    return false;
  }
  if (!ros::service::call(service, srv))
  {
    setStatus(QString("failed to call %1").arg(service.c_str()), true);
    return false;
  }
  return true;
}

void FlightTeleopPanel::sendGimbalZero()
{
  if (gimbals_.empty() && !loadGimbalConfig(robot_ns_))
  {
    setStatus(QString("no gimbal in %1/servo_controller/gimbals").arg(robot_ns_.c_str()), true);
    return;
  }
  if (gimbal_ctrl_pub_.getNumSubscribers() == 0)
  {
    setStatus(QString("nobody subscribes %1").arg(gimbal_ctrl_pub_.getTopic().c_str()), true);
    return;
  }
  if (!setGimbalTorque(true))
    return;
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  for (const auto& servo : gimbals_)
  {
    msg.name.push_back(servo.name);
    msg.position.push_back(0.0);
  }
  gimbal_ctrl_pub_.publish(msg);
  setStatus("gimbal torque on and 0 deg sent", false);
}

void FlightTeleopPanel::setGimbalZeroHere()
{
  if (gimbals_.empty() && !loadGimbalConfig(robot_ns_))
  {
    setStatus(QString("no gimbal in %1/servo_controller/gimbals").arg(robot_ns_.c_str()), true);
    return;
  }
  const auto indices = selectedGimbals();
  QStringList lines;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i : indices)
    {
      const auto it = joint_angles_.find(gimbals_[i].name);
      lines << QString("%1 (id %2): now %3 deg")
                   .arg(gimbals_[i].name.c_str())
                   .arg(gimbals_[i].id)
                   .arg(it != joint_angles_.end() ? QString::number(it->second * 180.0 / M_PI, 'f', 1) : "?");
    }
  }
  const auto answer = QMessageBox::question(
      this, "Set current as 0 deg",
      QString("The torque of the gimbals is turned off, and the homing offset of these servos is rewritten so that "
              "their present pose reads 0 deg. This is kept in the servos.\n\n%1\n\nHold each rotor level before "
              "pressing OK.")
          .arg(lines.join("\n")),
      QMessageBox::Ok | QMessageBox::Cancel, QMessageBox::Cancel);
  if (answer != QMessageBox::Ok)
    return;

  if (!setGimbalTorque(false))
    return;
  std::this_thread::sleep_for(std::chrono::duration<double>(TORQUE_OFF_WAIT));

  const std::string service = robot_ns_ + "/set_board_config";
  if (!ros::service::waitForService(service, ros::Duration(SERVICE_TIMEOUT)))
  {
    setStatus(QString("no service %1").arg(service.c_str()), true);
    return;
  }
  QStringList failed;
  for (int i : indices)
  {
    spinal::SetBoardConfig srv;
    srv.request.command = spinal::SetBoardConfig::Request::SET_SERVO_HOMING_OFFSET;
    // [slave_id (0: spinal), servo_index, raw value that the present pose should read]
    srv.request.data = { 0, gimbals_[i].id, gimbals_[i].zero_point_offset };
    if (!ros::service::call(service, srv) || !srv.response.success)
      failed << QString::fromStdString(gimbals_[i].name);
  }
  if (failed.empty())
    setStatus(QString("set the current pose as 0 deg for %1 (torque is off)").arg(gimbal_combo_->currentText()),
              false);
  else
    setStatus(QString("spinal refused the homing offset of %1 (torque still on?)").arg(failed.join(", ")), true);
}

void FlightTeleopPanel::update()
{
  int flight_state = -1;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto now = ros::WallTime::now();
    auto fresh = [&now](const ros::WallTime& stamp) {
      return !stamp.isZero() && (now - stamp).toSec() < STATE_TIMEOUT;
    };
    if (fresh(last_flight_state_time_))
      flight_state = flight_state_;
    position_label_->setText(fresh(last_odom_time_) ?
                                 QString("pos: %1, %2, %3 m")
                                     .arg(pos_x_, 0, 'f', 2)
                                     .arg(pos_y_, 0, 'f', 2)
                                     .arg(pos_z_, 0, 'f', 2) :
                                 "pos: ?");
    battery_label_->setText(fresh(last_battery_time_) ? QString("battery: %1 V").arg(battery_voltage_, 0, 'f', 1) :
                                                        "battery: ?");
    QStringList gimbal_texts;
    for (const auto& servo : gimbals_)
    {
      const auto it = joint_angles_.find(servo.name);
      QString text = QString("%1: %2")
                         .arg(servo.name.c_str())
                         .arg(fresh(last_joint_state_time_) && it != joint_angles_.end() ?
                                  QString("%1 deg").arg(it->second * 180.0 / M_PI, 0, 'f', 1) :
                                  "?");
      if (fresh(last_torque_state_time_) && servo.id >= 0 && servo.id < static_cast<int>(torque_states_.size()))
        text += torque_states_[servo.id] ? " (on)" : " (off)";
      gimbal_texts << text;
    }
    gimbal_label_->setText(gimbals_.empty() ? QString("no %1/servo_controller/gimbals").arg(robot_ns_.c_str()) :
                                              gimbal_texts.join(",  "));
  }
  if (flight_state < 0)
  {
    state_label_->setText("no flight_state");
    state_label_->setStyleSheet("QLabel { background-color: gray; color: white; }");
  }
  else
  {
    state_label_->setText(flightStateName(flight_state));
    state_label_->setStyleSheet(QString("QLabel { background-color: %1; color: white; }").arg(flightStateColor(flight_state)));
  }

  /* the gimbals must not be touched while the motors may spin */
  const bool motors_stopped = flight_state == ARM_OFF_STATE;
  for (auto button : { gimbal_on_button_, gimbal_off_button_, gimbal_zero_button_, gimbal_calib_button_ })
    button->setEnabled(motors_stopped);

  start_button_->setEnabled(enable_flight_check_->isChecked());
  takeoff_button_->setEnabled(enable_flight_check_->isChecked());

  const bool hovering = flight_state == HOVER_STATE;
  for (auto button : direction_buttons_)
    button->setEnabled(hovering);
  stop_button_->setEnabled(hovering);
  if (!hovering)
  {
    xy_active_ = z_active_ = yaw_active_ = false;
    return;
  }

  const double vx = (forward_button_->isDown() ? 1.0 : 0.0) - (backward_button_->isDown() ? 1.0 : 0.0);
  const double vy = (left_button_->isDown() ? 1.0 : 0.0) - (right_button_->isDown() ? 1.0 : 0.0);
  const double vz = (up_button_->isDown() ? 1.0 : 0.0) - (down_button_->isDown() ? 1.0 : 0.0);
  const double wz = (yaw_left_button_->isDown() ? 1.0 : 0.0) - (yaw_right_button_->isDown() ? 1.0 : 0.0);
  const bool xy_held = forward_button_->isDown() || backward_button_->isDown() || left_button_->isDown() ||
                       right_button_->isDown();
  const bool z_held = up_button_->isDown() || down_button_->isDown();
  const bool yaw_held = yaw_left_button_->isDown() || yaw_right_button_->isDown();

  /* a released axis gets zero velocity once */
  const bool send_xy = xy_held || xy_active_;
  const bool send_z = z_held || z_active_;
  const bool send_yaw = yaw_held || yaw_active_;
  xy_active_ = xy_held;
  z_active_ = z_held;
  yaw_active_ = yaw_held;
  if (!send_xy && !send_z && !send_yaw)
    return;

  aerial_robot_msgs::FlightNav nav;
  nav.header.stamp = ros::Time::now();
  nav.control_frame = frame_combo_->currentData().toInt();
  nav.target = aerial_robot_msgs::FlightNav::COG;
  if (send_xy)
  {
    const double norm = std::hypot(vx, vy);
    const double scale = norm > 0.0 ? xy_vel_spin_->value() / norm : 0.0;
    nav.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::VEL_MODE;
    nav.target_vel_x = vx * scale;
    nav.target_vel_y = vy * scale;
  }
  if (send_z)
  {
    nav.pos_z_nav_mode = aerial_robot_msgs::FlightNav::VEL_MODE;
    nav.target_vel_z = vz * z_vel_spin_->value();
  }
  if (send_yaw)
  {
    nav.yaw_nav_mode = aerial_robot_msgs::FlightNav::VEL_MODE;
    nav.target_omega_z = wz * yaw_vel_spin_->value() * M_PI / 180.0;
  }
  nav_pub_.publish(nav);
  status_label_->setStyleSheet("");
  status_label_->setText(QString("uav/nav vel: x %1, y %2, z %3 m/s, yaw %4 deg/s")
                             .arg(nav.target_vel_x, 0, 'f', 2)
                             .arg(nav.target_vel_y, 0, 'f', 2)
                             .arg(nav.target_vel_z, 0, 'f', 2)
                             .arg(nav.target_omega_z * 180.0 / M_PI, 0, 'f', 1));
}

void FlightTeleopPanel::stop()
{
  aerial_robot_msgs::FlightNav nav;
  nav.header.stamp = ros::Time::now();
  nav.control_frame = frame_combo_->currentData().toInt();
  nav.target = aerial_robot_msgs::FlightNav::COG;
  nav.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::VEL_MODE;
  nav.pos_z_nav_mode = aerial_robot_msgs::FlightNav::VEL_MODE;
  nav.yaw_nav_mode = aerial_robot_msgs::FlightNav::VEL_MODE;
  nav_pub_.publish(nav);
  status_label_->setStyleSheet("");
  status_label_->setText("sent zero velocity");
}
}  // namespace gimbalrotor_remote

PLUGINLIB_EXPORT_CLASS(gimbalrotor_remote::FlightTeleopPanel, rviz::Panel)
