#ifndef __VELOCITYCONTROLLER_H
#define __VELOCITYCONTROLLER_H

#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>
// ROS Stuff
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

// Messages
#include <myo_msgs/SetVelocity.h>
#include <myo_msgs/statusMessage.h>

namespace myo_controllers{

class velocityController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:
  double pwm;
  double ref;

  ros::ServiceServer srv_;
  realtime_tools::RealtimePublisher<myo_msgs::statusMessage> *realtime_pub;

  bool setVelocity( myo_msgs::SetVelocity::Request& req,
                    myo_msgs::SetVelocity::Response& resp)
  {
    if (fabs(req.velocity) < 80){
      ref = req.velocity;
      ROS_INFO("Change velocity to 100");
    } else
      ROS_WARN("Superduper error");

    resp.velocity = ref;
  }

  bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    ref = 50;
    pwm = 0;
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure

    srv_ = n.advertiseService("set_velocity", &velocityController::setVelocity, this);
    realtime_pub = new realtime_tools::RealtimePublisher<myo_msgs::statusMessage>(n, "DebugMessage", 4);

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    int hz = 1000;

    // get sensor values
    double position = joint_.getPosition();
    double velocity = joint_.getVelocity();
    double effort = joint_.getEffort();
    double displacement = joint_.getDisplacement();
    double analogIN0 = joint_.getAnalogIn(0);
    // set pwm cycle [-4000;4000]
    pwm = pwm -0.3*(velocity*hz/1000 - ref);
    if(abs(pwm) > 2000)
      pwm = 0.05*ref + pwm/abs(pwm) *2000;
    joint_.setCommand(pwm);
    if (realtime_pub->trylock()){
      /*  float64  dt
          float64  position
          float64  velocity
          float64  velocity_ref
          float64  commanded_effort
          float64  analogIN0          */
      realtime_pub->msg_.position         = position;
      realtime_pub->msg_.velocity         = velocity;
      realtime_pub->msg_.velocity_ref      = ref;
      realtime_pub->msg_.analogIN0         = analogIN0;
      realtime_pub->msg_.dt               = ros::Time::now();
      realtime_pub->msg_.commanded_effort = pwm;
      realtime_pub->unlockAndPublish();
    }
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

private:
  myo_interface::MyoMuscleJointHandle joint_;
};
}
#endif
