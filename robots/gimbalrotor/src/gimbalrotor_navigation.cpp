// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_navigation.h>
#include <std_msgs/String.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

GimbalrotorNavigator::GimbalrotorNavigator()
  : BaseNavigator(), eq_cog_world_(false), tilt_over_since_(-1), takeoff_ground_height_(0), prev_navi_state_(ARM_OFF_STATE)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void GimbalrotorNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);  // to spinal
  gimbal_zero_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbals_zero_sub_ = nh_.subscribe("gimbals_zero", 1, &GimbalrotorNavigator::gimbalsZeroCallback, this);
  gimbals_zeroed_ = false;
  supervisor_pub_ = nh_.advertise<std_msgs::String>("supervisor", 1, true);
  final_target_baselink_rot_sub_ =
      nh_.subscribe("final_target_baselink_rot", 1, &GimbalrotorNavigator::targetBaselinkRotCallback, this);
  final_target_baselink_rpy_sub_ =
      nh_.subscribe("final_target_baselink_rpy", 1, &GimbalrotorNavigator::targetBaselinkRPYCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();
}

void GimbalrotorNavigator::update()
{
  BaseNavigator::update();
  baselinkRotationProcess();
  supervise();
}

void GimbalrotorNavigator::supervisorAction(const std::string& reason, bool halt)
{
  std_msgs::String msg;
  msg.data = std::string(halt ? "halt: " : "force landing: ") + reason;
  supervisor_pub_.publish(msg);
  if (halt)
  {
    ROS_ERROR_STREAM("supervisor: " << reason << " -> motors off");
    setNaviState(STOP_STATE);
    return;
  }
  ROS_ERROR_STREAM("supervisor: " << reason << " -> force landing");
  spinal::FlightConfigCmd cmd;
  cmd.cmd = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
  flight_config_pub_.publish(cmd);
  force_landing_flag_ = true;
}

void GimbalrotorNavigator::supervise()
{
  const uint8_t state = getNaviState();
  if (state == TAKEOFF_STATE && prev_navi_state_ != TAKEOFF_STATE)
    takeoff_ground_height_ = estimator_->getPos(Frame::COG, estimate_mode_).z();
  prev_navi_state_ = state;
  if (!supervisor_enable_)
    return;
  if (state != TAKEOFF_STATE && state != HOVER_STATE && state != LAND_STATE)
  {
    tilt_over_since_ = -1;
    return;
  }

  const tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  const tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  const tf::Vector3 rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
  const double tilt = acos(std::max(-1.0, std::min(1.0, cos(rpy.x()) * cos(rpy.y()))));
  const double height = pos.z() - takeoff_ground_height_;
  const double now = ros::Time::now().toSec();
  char buf[160];

  /* halt: the crash is certain */
  if (tilt > halt_tilt_)
  {
    snprintf(buf, sizeof(buf), "tilt %.0f deg > %.0f deg", tilt * 180 / M_PI, halt_tilt_ * 180 / M_PI);
    supervisorAction(buf, true);
    return;
  }
  if (height < -halt_fall_height_ && vel.z() < -0.5)
  {
    snprintf(buf, sizeof(buf), "%.2f m below the takeoff spot and falling at %.1f m/s", -height, -vel.z());
    supervisorAction(buf, true);
    return;
  }

  /* takeoff abort: still over the legs, so cutting the thrust is the safe reaction */
  if (state == TAKEOFF_STATE && height < takeoff_abort_height_ && tilt > takeoff_abort_tilt_)
  {
    snprintf(buf, sizeof(buf), "takeoff aborted: tilt %.0f deg > %.0f deg at %.2f m", tilt * 180 / M_PI,
             takeoff_abort_tilt_ * 180 / M_PI, height);
    supervisorAction(buf, true);
    return;
  }

  if (force_landing_flag_)
    return;

  /* force landing: still flying, but not as commanded */
  if (tilt > force_landing_tilt_)
  {
    if (tilt_over_since_ < 0)
      tilt_over_since_ = now;
    if (now - tilt_over_since_ > force_landing_tilt_time_)
    {
      snprintf(buf, sizeof(buf), "tilt %.0f deg > %.0f deg for %.1f s", tilt * 180 / M_PI, force_landing_tilt_ * 180 / M_PI,
               now - tilt_over_since_);
      supervisorAction(buf, false);
      return;
    }
  }
  else
    tilt_over_since_ = -1;

  const tf::Vector3 delta = getTargetPos() - pos;
  const double err_xy = hypot(delta.x(), delta.y());
  if (err_xy > max_pos_error_xy_)
  {
    snprintf(buf, sizeof(buf), "xy error %.2f m > %.2f m", err_xy, max_pos_error_xy_);
    supervisorAction(buf, false);
    return;
  }
  if (state == HOVER_STATE && fabs(delta.z()) > max_pos_error_z_)
  {
    snprintf(buf, sizeof(buf), "z error %.2f m > %.2f m", delta.z(), max_pos_error_z_);
    supervisorAction(buf, false);
    return;
  }
  const double speed = vel.length();
  if (speed > max_speed_)
  {
    snprintf(buf, sizeof(buf), "speed %.1f m/s > %.1f m/s", speed, max_speed_);
    supervisorAction(buf, false);
    return;
  }
  if (height > max_height_)
  {
    snprintf(buf, sizeof(buf), "height %.2f m > %.2f m", height, max_height_);
    supervisorAction(buf, false);
    return;
  }
  if (geofence_.size() == 4 &&
      (pos.x() < geofence_[0] || pos.x() > geofence_[1] || pos.y() < geofence_[2] || pos.y() > geofence_[3]))
  {
    snprintf(buf, sizeof(buf), "outside the geofence: x %.2f y %.2f", pos.x(), pos.y());
    supervisorAction(buf, false);
    return;
  }
}

void GimbalrotorNavigator::gimbalsZeroCallback(const std_msgs::EmptyConstPtr& msg)
{
  sensor_msgs::JointState cmd;
  cmd.header.stamp = ros::Time::now();
  for (int i = 0; i < 4; i++)
  {
    cmd.name.push_back("gimbal" + std::to_string(i + 1));
    cmd.position.push_back(0.0);
  }
  gimbal_zero_pub_.publish(cmd);
  gimbals_zeroed_ = true;
  ROS_WARN("gimbals commanded to zero: check them, then takeoff");
}

bool GimbalrotorNavigator::takeoffAllowed()
{
  if (!takeoff_gimbal_check_)
    return true;
  if (!gimbals_zeroed_)
  {
    ROS_ERROR("takeoff refused: the gimbals have not been zeroed since arming. Publish gimbals_zero (std_msgs/Empty), "
              "look at the gimbals, then takeoff");
    return false;
  }
  const auto& joint_positions = robot_model_->getJointPositions();
  const auto& joint_index = robot_model_->getJointIndexMap();
  std::stringstream angles;
  bool ok = true;
  for (int i = 0; i < 4; i++)
  {
    const auto it = joint_index.find("gimbal" + std::to_string(i + 1));
    if (it == joint_index.end())
      continue;
    const double angle = joint_positions(it->second);
    angles << (i ? ", " : "") << angle * 180.0 / M_PI;
    if (fabs(angle) > takeoff_gimbal_tolerance_)
      ok = false;
  }
  if (!ok)
    ROS_ERROR_STREAM("takeoff refused: gimbal angles [" << angles.str() << "] deg are not within "
                     << takeoff_gimbal_tolerance_ * 180.0 / M_PI << " deg of zero. Publish gimbals_zero and check them");
  return ok;
}

void GimbalrotorNavigator::reset()
{
  gimbals_zeroed_ = false;
  BaseNavigator::reset();

  // a roll / pitch target given through uav/nav (e.g. script/excite_attitude.py) must not stay for the next flight
  setTargetRoll(0);
  setTargetPitch(0);

  // reset SO3
  eq_cog_world_ = false;
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);
}

void GimbalrotorNavigator::targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity

  // special process
  if (getTargetRPY().z() != 0)
  {
    curr_target_baselink_rot_.setRPY(0, 0, getTargetRPY().z());
    eq_cog_world_ = true;
  }
}

void GimbalrotorNavigator::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity
}

void GimbalrotorNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  BaseNavigator::naviCallback(msg);
  if (msg->roll_nav_mode == 2)
    setTargetRoll(msg->target_roll);
  if (msg->pitch_nav_mode == 2)
    setTargetPitch(msg->target_pitch);
}

void GimbalrotorNavigator::baselinkRotationProcess()
{
  if (curr_target_baselink_rot_ == final_target_baselink_rot_)
    return;

  if (ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
  {
    tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
    double angle = delta_q.getAngle();
    if (angle > M_PI)
      angle -= 2 * M_PI;

    if (fabs(angle) > baselink_rot_change_thresh_)
    {
      curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
    }
    else
      curr_target_baselink_rot_ = final_target_baselink_rot_;

    KDL::Rotation rot;
    tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
    robot_model_->setCogDesireOrientation(rot);

    // send to spinal
    spinal::DesireCoord msg;
    double r, p, y;
    tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
    msg.roll = r;
    msg.pitch = p;
    msg.yaw = y;
    target_baselink_rpy_pub_.publish(msg);

    prev_rotation_stamp_ = ros::Time::now().toSec();
  }
}

void GimbalrotorNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<bool>(navi_nh, "takeoff_gimbal_check", takeoff_gimbal_check_, true);
  getParam<double>(navi_nh, "takeoff_gimbal_tolerance", takeoff_gimbal_tolerance_, 5.0 * M_PI / 180.0);

  ros::NodeHandle sup_nh(navi_nh, "supervisor");
  getParam<bool>(sup_nh, "enable", supervisor_enable_, true);
  getParam<double>(sup_nh, "takeoff_abort_height", takeoff_abort_height_, 0.2);
  getParam<double>(sup_nh, "takeoff_abort_tilt", takeoff_abort_tilt_, 12.0 * M_PI / 180.0);
  getParam<double>(sup_nh, "force_landing_tilt", force_landing_tilt_, 30.0 * M_PI / 180.0);
  getParam<double>(sup_nh, "force_landing_tilt_time", force_landing_tilt_time_, 0.3);
  getParam<double>(sup_nh, "max_pos_error_xy", max_pos_error_xy_, 1.0);
  getParam<double>(sup_nh, "max_pos_error_z", max_pos_error_z_, 0.7);
  getParam<double>(sup_nh, "max_speed", max_speed_, 2.0);
  getParam<double>(sup_nh, "max_height", max_height_, 2.0);
  getParam<double>(sup_nh, "halt_tilt", halt_tilt_, 60.0 * M_PI / 180.0);
  getParam<double>(sup_nh, "halt_fall_height", halt_fall_height_, 0.1);
  sup_nh.param("geofence", geofence_, std::vector<double>());
  if (!geofence_.empty() && geofence_.size() != 4)
  {
    ROS_ERROR("navigation/supervisor/geofence needs 4 values [x_min, x_max, y_min, y_max], ignored");
    geofence_.clear();
  }
  ROS_INFO("flight supervisor %s: takeoff abort %.0f deg below %.2f m, force landing tilt %.0f deg / %.1f s, xy %.1f m, "
           "z %.1f m, speed %.1f m/s, height %.1f m%s; halt tilt %.0f deg",
           supervisor_enable_ ? "on" : "OFF", takeoff_abort_tilt_ * 180 / M_PI, takeoff_abort_height_,
           force_landing_tilt_ * 180 / M_PI, force_landing_tilt_time_, max_pos_error_xy_, max_pos_error_z_, max_speed_,
           max_height_, geofence_.empty() ? "" : ", geofence", halt_tilt_ * 180 / M_PI);
  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_,
                   0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_,
                   0.1);  // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorNavigator, aerial_robot_navigation::BaseNavigator);
