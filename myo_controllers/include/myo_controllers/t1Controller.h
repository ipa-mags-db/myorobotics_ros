#ifndef __DIRECTCONTROLLER_H
#define __DIRECTCONTROLLER_H

#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>
// ROS Stuff
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

// Messages
#include <myo_msgs/SetReference.h>
#include <myo_msgs/SetClutch.h>
#include <myo_msgs/SetPID.h>
#include <myo_msgs/statusMessage.h>

#include <sstream>
#include <string>

#define MAXCUR 50000
#define MAXREF 3999
#define MAXPWM 3999


namespace myo_controllers{

class t1Controller : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:
  int count;
  std::string s;
  std::stringstream ss;

  bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n)
  {
    std::string my_joint;
    if (!n.getParam("joint", my_joint)) {
            ROS_ERROR("Could not find joint name");
            return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);

    s = " ";
    count = 0;
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // TODO make hz meaningful
    int hz = 1000;

    // get sensor values
    double position       = joint_.getPosition();
    double velocity       = joint_.getVelocity();
    double effort         = joint_.getEffort();
    double displacement   = joint_.getDisplacement();
    double analogIN0      = joint_.getAnalogIn(0);

    if(count%10000){

      for(int i = 0 ; i<6; ++i)
        ss << std::hex << (int)joint_.getAnalogIn(i) << " ";
      s = ss.str();
      ROS_INFO_STREAM(s);
      s.clear();
      ss.str("");

    }


    count++;
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) {
  }

private:
  myo_interface::MyoMuscleJointHandle joint_;
};
}
#endif
