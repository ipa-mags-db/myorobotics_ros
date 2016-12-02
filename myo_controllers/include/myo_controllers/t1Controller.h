#ifndef __DIRECTCONTROLLER_H
#define __DIRECTCONTROLLER_H

#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>
// ROS Stuff
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <stdint.h>

// Messages
#include <myo_msgs/analogMessage.h>

#include <sstream>
#include <string>

namespace myo_controllers{

class t1Controller : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:
  int count;
  std::string s;
  std::stringstream ss;
  double analogIN[6];
  realtime_tools::RealtimePublisher<myo_msgs::analogMessage> *realtime_pub;

  bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n)
  {
    std::string my_joint;
    if (!n.getParam("joint", my_joint)) {
            ROS_ERROR("Could not find joint name");
            return false;
    }

    clock_gettime(CLOCK_MONOTONIC, &this->no);

    // get the joint object to use in the realtime loops
    joint_ = hw->getHandle(my_joint);

    realtime_pub = new realtime_tools::RealtimePublisher<myo_msgs::analogMessage>(n, "AnalogDataMessage", 25);

    s = " ";
    count = 0;
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // TODO make hz meaningful
    int hz = 1000;

    // get sensor values

    for( int i = 0; i<6  ; ++i)
          analogIN[i] =(double)(int16_t)joint_.getAnalogIn(i);

    if(count%(5*1000)==0){

      for(int i = 0 ; i<6; ++i)
        ss << (int16_t)joint_.getAnalogIn(i) << " ";
      s = ss.str();
      ROS_INFO_STREAM(s);
      s.clear();
      ss.str("");
    }
    clock_gettime(CLOCK_MONOTONIC, &this->no);

    if (realtime_pub->trylock()) {

            ros::Time t1(no.tv_sec, no.tv_nsec);

            realtime_pub->msg_.time       = t1;


            realtime_pub->msg_.analogIN0  = (double)analogIN[0];
            realtime_pub->msg_.analogIN1  = (double)analogIN[1];
            realtime_pub->msg_.analogIN2  = (double)analogIN[2];
            realtime_pub->msg_.analogIN3  = (double)analogIN[3];
            realtime_pub->msg_.analogIN4  = (double)analogIN[4];
            realtime_pub->msg_.analogIN5  = (double)analogIN[5];

            realtime_pub->unlockAndPublish();

    };



    count++;
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) {
  }


private:
  myo_interface::MyoMuscleJointHandle joint_;
  struct timespec no;
};
}
#endif
