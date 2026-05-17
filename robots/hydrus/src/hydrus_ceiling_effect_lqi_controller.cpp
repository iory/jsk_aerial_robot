#include <hydrus/hydrus_ceiling_effect_lqi_controller.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

using namespace aerial_robot_control;

namespace
{
std::string trimString(const std::string& s)
{
  const auto first = s.find_first_not_of(" \t\r\n");
  if(first == std::string::npos) return "";

  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::vector<std::string> splitCSVLine(const std::string& line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;

  while(std::getline(ss, token, ','))
  {
    tokens.push_back(trimString(token));
  }

  return tokens;
}
} // namespace

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
  
  nh.param("rotor_radius", rotor_radius_, 0.1905);
  nh.param("ceiling_height", ceiling_height_, 1.5);//ceiling_height by yourself
  nh.param("ceiling_distance_offset", ceiling_distance_offset_, 0.05);
  // 0: no compensation
  // 1: single-rotor mode, use l_bar = 10.0
  // 2: multi-rotor mode, use l12_bar and l34_bar
  nh.param("ceiling_effect_mode", ceiling_effect_mode_, 0);
  nh.param<std::string>("ceiling_effect_table_path", ceiling_effect_table_path_, std::string(""));

  if(ceiling_effect_mode_ < 0 || ceiling_effect_mode_ > 2)
  {
    ROS_WARN("invalid ceiling_effect_mode: %d. Use mode 0.", ceiling_effect_mode_);
    ceiling_effect_mode_ = 0;
  }

  if(ceiling_effect_mode_ > 0)
  {
    if(ceiling_effect_table_path_.empty())
    {
      ROS_ERROR("ceiling_effect_table_path is empty. Disable ceiling effect compensation.");
      ceiling_effect_mode_ = 0;
    }
    else if(!loadCTRatioTable(ceiling_effect_table_path_))
    {
      ROS_ERROR("failed to load ceiling effect table: %s. Disable ceiling effect compensation.",
                ceiling_effect_table_path_.c_str());
      ceiling_effect_mode_ = 0;
    }
  }
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

    // ceiling distance from rotor disk/reference point.
    ceiling_distance_ = ceiling_height_ - root_z - ceiling_distance_offset_;
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
  // ceiling_effect_gain_ stores CT_ratio = k.
  // updateCeilingEffectParams() publishes 1.0 / k to spinal.

  for(int i = 0; i < motor_num_; ++i)
  {
    ceiling_effect_gain_.at(i) = 1.0;
  }

  if(ceiling_effect_mode_ == 0)
  {
    return;
  }

  const double d_R = ceiling_distance_ratio_;

  if(ceiling_effect_mode_ == 1)
  {
    const double l_bar_single = 10.0;
    const double k = lookupCTRatio(l_bar_single, d_R);

    for(int i = 0; i < motor_num_; ++i)
    {
      ceiling_effect_gain_.at(i) = k;
    }
    return;
  }

  if(ceiling_effect_mode_ == 2)
  {
    if(rotor_distance_ratio_.size() < 6 || motor_num_ < 4)
    {
      ROS_WARN_THROTTLE(1.0, "invalid rotor_distance_ratio_ size or motor_num_. Use no compensation.");
      return;
    }

    const double l12_bar = rotor_distance_ratio_.at(0); // l12/R - 2
    const double l34_bar = rotor_distance_ratio_.at(5); // l34/R - 2

    const double k12 = lookupCTRatio(l12_bar, d_R);
    const double k34 = lookupCTRatio(l34_bar, d_R);

    ceiling_effect_gain_.at(0) = k12;
    ceiling_effect_gain_.at(1) = k12;
    ceiling_effect_gain_.at(2) = k34;
    ceiling_effect_gain_.at(3) = k34;

    return;
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

int HydrusCeilingEffectLQIController::ratioKey(double value)
{
  return static_cast<int>(std::round(value * 100.0));
}

bool HydrusCeilingEffectLQIController::loadCTRatioTable(const std::string& csv_path)
{
  std::ifstream ifs(csv_path.c_str());
  if(!ifs.is_open())
  {
    ROS_ERROR("cannot open ceiling effect table csv: %s", csv_path.c_str());
    return false;
  }

  std::string line;
  if(!std::getline(ifs, line))
  {
    ROS_ERROR("empty ceiling effect table csv: %s", csv_path.c_str());
    return false;
  }

  const std::vector<std::string> header = splitCSVLine(line);

  int l_idx = -1;
  int d_idx = -1;
  int ratio_idx = -1;

  for(size_t i = 0; i < header.size(); ++i)
  {
    if(header.at(i) == "l_R") l_idx = static_cast<int>(i);
    else if(header.at(i) == "d_R") d_idx = static_cast<int>(i);
    else if(header.at(i) == "CT_ratio") ratio_idx = static_cast<int>(i);
  }

  if(l_idx < 0 || d_idx < 0 || ratio_idx < 0)
  {
    ROS_ERROR("csv header must contain l_R, d_R, CT_ratio");
    return false;
  }

  ct_ratio_table_.clear();
  ct_l_grid_.clear();
  ct_d_grid_.clear();

  std::set<int> l_keys;
  std::set<int> d_keys;

  int loaded_count = 0;

  while(std::getline(ifs, line))
  {
    if(line.empty()) continue;

    const std::vector<std::string> tokens = splitCSVLine(line);
    if(static_cast<int>(tokens.size()) <= std::max({l_idx, d_idx, ratio_idx}))
    {
      continue;
    }

    try
    {
      const double l_bar = std::stod(tokens.at(l_idx));
      const double d_R = std::stod(tokens.at(d_idx));
      const double ct_ratio = std::stod(tokens.at(ratio_idx));

      if(!std::isfinite(l_bar) || !std::isfinite(d_R) || !std::isfinite(ct_ratio))
      {
        continue;
      }

      const int lk = ratioKey(l_bar);
      const int dk = ratioKey(d_R);

      ct_ratio_table_[std::make_pair(lk, dk)] = ct_ratio;
      l_keys.insert(lk);
      d_keys.insert(dk);
      ++loaded_count;
    }
    catch(const std::exception& e)
    {
      ROS_WARN("failed to parse csv line: %s", line.c_str());
      continue;
    }
  }

  for(const auto& k : l_keys)
  {
    ct_l_grid_.push_back(static_cast<double>(k) / 100.0);
  }

  for(const auto& k : d_keys)
  {
    ct_d_grid_.push_back(static_cast<double>(k) / 100.0);
  }

  if(ct_ratio_table_.empty() || ct_l_grid_.empty() || ct_d_grid_.empty())
  {
    ROS_ERROR("no valid data in ceiling effect table csv: %s", csv_path.c_str());
    return false;
  }

  ROS_INFO("loaded ceiling effect CT_ratio table: %d points, %zu l-grid, %zu d-grid",
           loaded_count, ct_l_grid_.size(), ct_d_grid_.size());

  return true;
}

double HydrusCeilingEffectLQIController::getCTRatioFromTable(double l_bar, double d_R) const
{
  const int lk = ratioKey(l_bar);
  const int dk = ratioKey(d_R);

  const auto it = ct_ratio_table_.find(std::make_pair(lk, dk));
  if(it == ct_ratio_table_.end())
  {
    ROS_WARN_THROTTLE(1.0,
                      "CT_ratio table value not found: l_bar=%.3f, d_R=%.3f. Use 1.0.",
                      l_bar, d_R);
    return 1.0;
  }

  return it->second;
}

double HydrusCeilingEffectLQIController::lookupCTRatio(double l_bar, double d_R) const
{
  if(ct_ratio_table_.empty() || ct_l_grid_.empty() || ct_d_grid_.empty())
  {
    return 1.0;
  }

  if(!std::isfinite(l_bar) || !std::isfinite(d_R))
  {
    return 1.0;
  }

  if(d_R > 1.99)
  {
    return 1.0;
  }

  d_R = std::max(0.30, d_R);
  l_bar = std::max(0.30, std::min(10.0, l_bar));

  d_R = std::max(ct_d_grid_.front(), std::min(ct_d_grid_.back(), d_R));
  l_bar = std::max(ct_l_grid_.front(), std::min(ct_l_grid_.back(), l_bar));

  auto findBracket = [](const std::vector<double>& grid, double x) -> std::pair<double, double>
  {
    auto upper_it = std::lower_bound(grid.begin(), grid.end(), x);

    if(upper_it == grid.begin())
    {
      return std::make_pair(*upper_it, *upper_it);
    }

    if(upper_it == grid.end())
    {
      return std::make_pair(grid.back(), grid.back());
    }

    if(std::abs(*upper_it - x) < 1e-9)
    {
      return std::make_pair(*upper_it, *upper_it);
    }

    return std::make_pair(*(upper_it - 1), *upper_it);
  };

  const auto l_bracket = findBracket(ct_l_grid_, l_bar);
  const auto d_bracket = findBracket(ct_d_grid_, d_R);

  const double l0 = l_bracket.first;
  const double l1 = l_bracket.second;
  const double d0 = d_bracket.first;
  const double d1 = d_bracket.second;

  const double Q00 = getCTRatioFromTable(l0, d0);
  const double Q10 = getCTRatioFromTable(l1, d0);
  const double Q01 = getCTRatioFromTable(l0, d1);
  const double Q11 = getCTRatioFromTable(l1, d1);

  const bool same_l = std::abs(l1 - l0) < 1e-9;
  const bool same_d = std::abs(d1 - d0) < 1e-9;

  if(same_l && same_d)
  {
    return Q00;
  }

  if(same_l)
  {
    const double b = (d_R - d0) / (d1 - d0);
    return Q00 * (1.0 - b) + Q01 * b;
  }

  if(same_d)
  {
    const double a = (l_bar - l0) / (l1 - l0);
    return Q00 * (1.0 - a) + Q10 * a;
  }

  const double a = (l_bar - l0) / (l1 - l0);
  const double b = (d_R - d0) / (d1 - d0);

  return Q00 * (1.0 - a) * (1.0 - b)
       + Q10 * a         * (1.0 - b)
       + Q01 * (1.0 - a) * b
       + Q11 * a         * b;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusCeilingEffectLQIController, aerial_robot_control::ControlBase);
