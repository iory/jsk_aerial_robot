// -*- mode: c++ -*-

#include <controller_manager/controller_manager.h>
#include <gimbalrotor/arm/arm_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_hardware_interface");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  double control_rate;
  std::string controller_ns;
  nhp.param("control_rate", control_rate, 20.0);
  nhp.param("controller_ns", controller_ns, std::string("arm"));

  /* one thread for joint_states, one for the services of controller_manager */
  ros::AsyncSpinner spinner(2);
  spinner.start();

  gimbalrotor::ArmHardwareInterface arm_hw;
  if (!arm_hw.init(nh, nhp))
  {
    ROS_FATAL("[arm hw] failed to initialize the hardware interface");
    return 1;
  }

  ros::NodeHandle controller_nh(nh, controller_ns);
  controller_manager::ControllerManager controller_manager(&arm_hw, controller_nh);

  ros::Rate rate(control_rate);
  ros::Time prev_time = ros::Time::now();
  bool started = false;
  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    const ros::Duration period = now - prev_time;
    prev_time = now;

    /*
     * Do not update the controllers until all joint positions are received,
     * otherwise controllers start (e.g. hold position) from the uninitialized position.
     * The switch requests from the spawner are processed after this.
     */
    if (!started)
    {
      if (!arm_hw.allJointStatesReceived())
      {
        ROS_WARN_THROTTLE(5.0, "[arm hw] waiting for joint_states of all arm joints");
        rate.sleep();
        continue;
      }
      ROS_INFO("[arm hw] all joint states are received, start controller update");
      started = true;
    }

    arm_hw.read(now, period);
    controller_manager.update(now, period);
    arm_hw.write(now, period);
    rate.sleep();
  }

  spinner.stop();
  return 0;
}
