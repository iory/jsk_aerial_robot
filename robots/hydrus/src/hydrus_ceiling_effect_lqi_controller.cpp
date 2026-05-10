#include <hydrus/hydrus_ceiling_effect_lqi_controller.h>

using namespace aerial_robot_control;

HydrusCeilingEffectLQIController::HydrusCeilingEffectLQIController():
  UnderActuatedLQIController()
{
}


void HydrusCeilingEffectLQIController::initialize(ros::NodeHandle nh,
                                                 ros::NodeHandle nhp,
                                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  
  nhp.param("rotor_radius", rotor_radius_, 0.1905);
  nhp.param("ceiling_height", ceiling_height_, 1.5);//ceiling_height by yourself
  nh.param("ceiling_effect", ceiling_effect_enabled_, false);
  rotor_distance_.assign(6, 0.0);
  rotor_distance_ratio_.assign(6, 0.0);

  ceiling_effect_gain_.assign(motor_num_, 1.0);

  // ceiling effect debug publisher
  ceiling_distance_ratio_pub_ = nh.advertise<std_msgs::Float64>("ceiling_effect/ceiling_distance_ratio", 1);
  rotor_distance_ratio_pub_ = nh.advertise<std_msgs::Float64MultiArray>("ceiling_effect/rotor_distance_ratio", 1);
  ceiling_effect_thrust_ratio_pub_ = nh.advertise<std_msgs::Float32MultiArray>("ceiling_effect/thrust_ratio", 1);
}

bool HydrusCeilingEffectLQIController::checkRobotModel()
{
  boost::shared_ptr<HydrusRobotModel> hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model_);
  lqi_mode_ = hydrus_robot_model->getWrenchDof();

  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");
      if(hydrus_robot_model->getWrenchDof() == 4 && hydrus_robot_model->getFeasibleControlRollPitchMin() > hydrus_robot_model->getFeasibleControlRollPitchMinThre())
        {
          ROS_WARN_NAMED("LQI gain generator", "LQI gain generator: change to three axis stable mode");
          lqi_mode_ = 3;
          return true;
        }

      return false;
    }
  return true;
}

void HydrusCeilingEffectLQIController::compensateBaseThrust(std::vector<float>& compensated_base_thrust)
{
  for(int i = 0; i < motor_num_; ++i)
  {
    double k = ceiling_effect_gain_.at(i);

    if(k < 1e-6)
    {
      ROS_WARN_THROTTLE(1.0, "ceiling effect gain is too small: %f", k);
      k = 1.0;
    }

    compensated_base_thrust.at(i) = static_cast<float>(target_base_thrust_.at(i) / k);
  }
}

bool HydrusCeilingEffectLQIController::updateRotorDistances()
{
  std::vector<tf::Vector3> p(4);
  const std::vector<std::string> frames = {
      "hydrus/thrust1",
      "hydrus/thrust2",
      "hydrus/thrust3",
      "hydrus/thrust4"
  };

  for(int i = 0; i < 4; ++i)
  {
    tf::StampedTransform transform;
    try
    {
      tf_listener_.lookupTransform("hydrus/root", frames.at(i), ros::Time(0), transform);
      p.at(i) = transform.getOrigin();
    }
    catch(tf::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  const std::vector<std::pair<int, int>> pairs = {
      {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}
  };

  for(size_t n = 0; n < pairs.size(); ++n)
  {
    int i = pairs[n].first;
    int j = pairs[n].second;
    rotor_distance_.at(n) = (p.at(i) - p.at(j)).length();
    rotor_distance_ratio_.at(n) = rotor_distance_.at(n) / rotor_radius_ - 2.0; // bar{l} = l/R - 2
  }
  return true;
}

bool HydrusCeilingEffectLQIController::updateCeilingDistance()
{
  tf::StampedTransform transform;
  try
  {
    tf_listener_.lookupTransform("world", "hydrus/root", ros::Time(0), transform);
    double root_z = transform.getOrigin().z();
    ceiling_distance_ = ceiling_height_ - root_z;
    ceiling_distance_ratio_ = ceiling_distance_ / rotor_radius_;
    return true;
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
    return false;
  }
}

void HydrusCeilingEffectLQIController::updateCeilingEffectGain()
{
  if(!ceiling_effect_enabled_)
  {
    for(int i = 0; i < motor_num_; ++i)
    {
      ceiling_effect_gain_.at(i) = 1.0;
    }
    return;
  }

  for(int i = 0; i < motor_num_; ++i)
  {
    // TODO: replace with k(d,l)
    ceiling_effect_gain_.at(i) = 2.0;  // temporary test value
  }
}

bool HydrusCeilingEffectLQIController::updateCeilingEffectParams()
{
  if(!updateCeilingDistance()) return false;
  if(!updateRotorDistances()) return false;

  updateCeilingEffectGain();

  std_msgs::Float64 distance_ratio_msg;
  distance_ratio_msg.data = ceiling_distance_ratio_;
  ceiling_distance_ratio_pub_.publish(distance_ratio_msg);

  std_msgs::Float64MultiArray rotor_distance_ratio_msg;
  rotor_distance_ratio_msg.data.clear();
  for(const auto& l : rotor_distance_ratio_)
  rotor_distance_ratio_msg.data.push_back(l);
  rotor_distance_ratio_pub_.publish(rotor_distance_ratio_msg);

  std_msgs::Float32MultiArray ceiling_effect_thrust_ratio_msg;
  ceiling_effect_thrust_ratio_msg.data.clear();
  for(const auto& k : ceiling_effect_gain_)
 {
  float ratio = 1.0f;
  if(std::isfinite(k) && k > 1e-6)
  {
    ratio = static_cast<float>(1.0 / k);
  }
  ceiling_effect_thrust_ratio_msg.data.push_back(ratio);
  }
  ceiling_effect_thrust_ratio_pub_.publish(ceiling_effect_thrust_ratio_msg);

  return true;
}

void HydrusCeilingEffectLQIController::controlCore()
{
  updateCeilingEffectParams();

  UnderActuatedLQIController::controlCore();
}

void HydrusCeilingEffectLQIController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;

  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;

  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusCeilingEffectLQIController, aerial_robot_control::ControlBase);
