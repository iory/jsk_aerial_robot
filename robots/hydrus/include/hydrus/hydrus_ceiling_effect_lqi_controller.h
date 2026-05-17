// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <aerial_robot_control/control/under_actuated_lqi_controller.h>
#include <hydrus/hydrus_robot_model.h>
#include <tf/transform_listener.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <map>
#include <string>
#include <utility>

namespace aerial_robot_control
{
  class HydrusCeilingEffectLQIController: public UnderActuatedLQIController
  {

  public:
    HydrusCeilingEffectLQIController();
    virtual ~HydrusCeilingEffectLQIController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

  protected:

    bool checkRobotModel() override;
    
    //controller update()
    void controlCore() override;
    void sendFourAxisCommand() override;

    //ceiling effect related functions
    bool updateCeilingEffectParams();
    bool updateCeilingDistance();
    bool updateRotorDistances();
    void updateCeilingEffectGain();
    void compensateBaseThrust(std::vector<float>& compensated_base_thrust);
    
    //CT ratio table related functions
    bool loadCTRatioTable(const std::string& csv_path);
    double lookupCTRatio(double l_bar, double d_R) const;
    double getCTRatioFromTable(double l_bar, double d_R) const;
    static int ratioKey(double value);

    //get tf
    tf::TransformListener tf_listener_;

    // ceiling effect related parameters
    bool ceiling_effect_enabled_;  // old bool parameter, kept for compatibility
    int ceiling_effect_mode_;      // 0: none, 1: single-rotor, 2: multi-rotor
    std::string ceiling_effect_table_path_;

    double rotor_radius_;
    double ceiling_height_;
    double ceiling_distance_;
    double ceiling_distance_ratio_;

    std::vector<double> rotor_distance_;
    std::vector<double> rotor_distance_ratio_;

    // Stores CT_ratio = k(d_R, l_bar).
    // updateCeilingEffectParams() publishes 1.0 / k to spinal.
    std::vector<double> ceiling_effect_gain_;

    // CT_ratio table
    // key: round(value * 100), e.g., 0.30 -> 30, 10.0 -> 1000
    std::vector<double> ct_l_grid_;
    std::vector<double> ct_d_grid_;
    std::map<std::pair<int, int>, double> ct_ratio_table_;

    ros::Publisher ceiling_distance_ratio_pub_;
    ros::Publisher rotor_distance_ratio_pub_;
    ros::Publisher ceiling_effect_thrust_ratio_pub_;
  };
};
