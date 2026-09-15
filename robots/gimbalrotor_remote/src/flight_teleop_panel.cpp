// -*- mode: c++ -*-

#include "flight_teleop_panel.h"

#include <aerial_robot_msgs/FlightNav.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Empty.h>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <cmath>

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

/* aerial_robot_navigation::BaseNavigator::flight_state and the special states of flight_state */
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
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_flight_state_time_ = ros::WallTime();
    last_odom_time_ = ros::WallTime();
    last_battery_time_ = ros::WallTime();
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
