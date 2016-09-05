#ifndef __EXAMPLECONTROLLER_H
#define __EXAMPLECONTROLLER_H

#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <myo_msgs/SetVelocity.h>

namespace myo_controllers{

class ExampleController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:
  double pwm;
  double ref;

  ros::ServiceServer srv_;

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

    srv_ = n.advertiseService("set_velocity", &ExampleController::setVelocity, this);

    joint_.setDigitalOut(true);
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    int hz = 1000;

    ros::Rate r(hz);
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
    r.sleep();
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

private:
  myo_interface::MyoMuscleJointHandle joint_;
};
}
#endif
