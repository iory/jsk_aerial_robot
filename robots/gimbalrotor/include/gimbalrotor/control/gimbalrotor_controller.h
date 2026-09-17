// -*- mode: c++ -*-

#pragma once

#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>

namespace aerial_robot_control
{
class GimbalrotorController : public PoseLinearController
{
public:
  GimbalrotorController();
  ~GimbalrotorController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

private:
  ros::Publisher flight_cmd_pub_;
  ros::Publisher gimbal_control_pub_;
  ros::Publisher gimbal_state_pub_;
  ros::Publisher target_vectoring_force_pub_;
  ros::Publisher rpy_gain_pub_;                      // for spinal
  ros::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros::Publisher gimbal_dof_pub_;                    // for spinal

  boost::shared_ptr<GimbalrotorRobotModel> gimbalrotor_robot_model_;
  std::vector<float> target_base_thrust_;
  std::vector<float> target_full_thrust_;
  std::vector<double> target_gimbal_angles_;
  bool hovering_approximate_;
  Eigen::VectorXd target_vectoring_f_;
  Eigen::VectorXd target_vectoring_f_trans_;
  Eigen::VectorXd target_vectoring_f_rot_;
  Eigen::MatrixXd integrated_map_inv_trans_;
  Eigen::MatrixXd integrated_map_inv_rot_;
  double candidate_yaw_term_;
  int gimbal_dof_;
  int rotor_coef_;
  bool gimbal_calc_in_fc_;
  bool underactuate_;
  double target_roll_ = 0.0, target_pitch_ = 0.0;
  /* gimbal lag compensation: the thrusts are re-allocated on the measured gimbal angles, so that the fast
     thrusts give the z force and the roll / pitch torque the gimbals do not give yet */
  bool gimbal_lag_compensation_ = false;
  double gimbal_lag_compensation_limit_ = 2.0;  // [N] per rotor
  ros::Publisher thrust_correction_pub_;
  void compensateGimbalLag(const Eigen::VectorXd& target_wrench_acc_cog, const Eigen::Matrix3d& inertia_inv,
                           double mass_inv);

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
};
};  // namespace aerial_robot_control
