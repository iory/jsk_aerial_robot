// -*- mode: c++ -*-
/*
 * Gazebo classic drops the <mimic> tags of a URDF when it converts the model to SDF,
 * so a mimic joint is left unactuated. This plugin reads the URDF from the parameter
 * server and drives every mimic joint to multiplier * position(mimicked joint) + offset
 * on each world update.
 *
 * The joint is driven by the velocity motor of the physics engine (a velocity
 * proportional to the position error, limited by the effort of the joint), not by
 * setting its position: moving a link of an articulated chain by hand on every step
 * feeds energy into the solver. With SetPosition, the open gripper of grape_with_arm
 * made the arm joints of the hovering robot break out by up to 145 deg every ~38 s.
 *
 * <gazebo>
 *   <plugin name="mimic_joint_plugin" filename="libaerial_robot_mimic_joint_plugin.so">
 *     <robotNamespace>gimbalrotor</robotNamespace>
 *     <robotParam>robot_description</robotParam>  <!-- optional -->
 *     <positionGain>20.0</positionGain>           <!-- optional, [1/s] -->
 *   </plugin>
 * </gazebo>
 */

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <ros/ros.h>
#include <urdf/model.h>

namespace aerial_robot_simulation
{
  class MimicJointPlugin : public gazebo::ModelPlugin
  {
  public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      if (!ros::isInitialized())
        {
          ROS_FATAL_STREAM("[mimic joint] ROS is not initialized, load gazebo with libgazebo_ros_api_plugin.so");
          return;
        }

      const std::string robot_namespace = sdf->HasElement("robotNamespace") ? sdf->Get<std::string>("robotNamespace") : "";
      const std::string robot_param = sdf->HasElement("robotParam") ? sdf->Get<std::string>("robotParam") : "robot_description";
      position_gain_ = sdf->HasElement("positionGain") ? sdf->Get<double>("positionGain") : 20.0;
      ros::NodeHandle nh(robot_namespace);

      std::string urdf_string;
      if (!nh.getParam(robot_param, urdf_string))
        {
          ROS_ERROR_STREAM("[mimic joint] can not find rosparam " << nh.resolveName(robot_param));
          return;
        }
      urdf::Model urdf_model;
      if (!urdf_model.initString(urdf_string))
        {
          ROS_ERROR_STREAM("[mimic joint] failed to parse " << nh.resolveName(robot_param));
          return;
        }

      for (const auto& joint_pair : urdf_model.joints_)
        {
          const urdf::JointConstSharedPtr& urdf_joint = joint_pair.second;
          if (!urdf_joint->mimic)
            continue;

          MimicJoint mimic_joint;
          mimic_joint.joint = model->GetJoint(urdf_joint->name);
          mimic_joint.mimicked = model->GetJoint(urdf_joint->mimic->joint_name);
          if (!mimic_joint.joint || !mimic_joint.mimicked)
            {
              ROS_ERROR_STREAM("[mimic joint] " << urdf_joint->name << " mimics " << urdf_joint->mimic->joint_name
                               << ", but " << (mimic_joint.joint ? urdf_joint->mimic->joint_name : urdf_joint->name)
                               << " does not exist in gazebo (a fixed joint is merged away)");
              continue;
            }
          mimic_joint.multiplier = urdf_joint->mimic->multiplier;
          mimic_joint.offset = urdf_joint->mimic->offset;
          mimic_joint.max_velocity = mimic_joint.joint->GetVelocityLimit(0);
          if (mimic_joint.max_velocity <= 0.0)
            mimic_joint.max_velocity = std::numeric_limits<double>::infinity();
          // the motor holds the joint against the load up to this force
          const double effort = mimic_joint.joint->GetEffortLimit(0);
          mimic_joint.motor = effort > 0.0 && mimic_joint.joint->SetParam("fmax", 0, effort);
          if (!mimic_joint.motor)
            ROS_WARN_STREAM("[mimic joint] " << urdf_joint->name << " has no usable velocity motor "
                            << "(effort limit " << effort << "); its position is set directly, "
                            << "which can destabilize the simulation");
          mimic_joints_.push_back(mimic_joint);
          ROS_INFO_STREAM("[mimic joint] " << urdf_joint->name << " = " << mimic_joint.multiplier << " * "
                          << urdf_joint->mimic->joint_name << " + " << mimic_joint.offset);
        }

      if (mimic_joints_.empty())
        {
          ROS_WARN_STREAM("[mimic joint] no mimic joint in " << nh.resolveName(robot_param));
          return;
        }
      update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MimicJointPlugin::update, this));
    }

  private:
    struct MimicJoint
    {
      gazebo::physics::JointPtr joint;
      gazebo::physics::JointPtr mimicked;
      double multiplier;
      double offset;
      double max_velocity;
      bool motor;
    };

    void update()
    {
      for (const auto& mimic_joint : mimic_joints_)
        {
          double target = mimic_joint.multiplier * mimic_joint.mimicked->Position(0) + mimic_joint.offset;
          target = std::clamp(target, mimic_joint.joint->LowerLimit(0), mimic_joint.joint->UpperLimit(0));
          if (!mimic_joint.motor)
            {
              mimic_joint.joint->SetPosition(0, target, true);
              continue;
            }
          const double error = target - mimic_joint.joint->Position(0);
          const double velocity = std::clamp(position_gain_ * error,
                                             -mimic_joint.max_velocity, mimic_joint.max_velocity);
          mimic_joint.joint->SetParam("vel", 0, velocity);
        }
    }

    std::vector<MimicJoint> mimic_joints_;
    double position_gain_;
    gazebo::event::ConnectionPtr update_connection_;
  };
}  // namespace aerial_robot_simulation

GZ_REGISTER_MODEL_PLUGIN(aerial_robot_simulation::MimicJointPlugin)
