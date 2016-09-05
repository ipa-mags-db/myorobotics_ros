#ifndef __testController_H
#define __testController_H


#define STARTUP 0
#define RUN 1
#define CURRENTMAX 200
#define CHANGED 1
#define UNCHANGED 0

#include <time.h>
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

class testController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:
  double pwm;
  double ref;
  struct timespec no;
  struct timespec b;
  double beforePos;
  int state;
  int maxpos;
  int minpos;
  int direction;
  double current;

  ros::ServiceServer srv_;
  realtime_tools::RealtimePublisher<myo_msgs::statusMessage> *realtime_pub;

  bool setVelocity( myo_msgs::SetVelocity::Request& req,
                    myo_msgs::SetVelocity::Response& resp)
  {
    if (fabs(req.velocity) < 5000){
      ref = req.velocity;
      ROS_INFO("Change velocity to %f",ref);
    } else
      ROS_WARN("Superduper error");

    resp.velocity = ref;
  }

  bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    ref = 10;
    pwm = 0;
    current = 0;
    state = STARTUP;
    direction = UNCHANGED;


    clock_gettime(CLOCK_MONOTONIC,&no);
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure

    // create new service for setting the velocity and bind setVelocity as callback
    srv_ = n.advertiseService("set_velocity", &testController::setVelocity, this);
    // create realtime publisher with a buffer for 25 messages
    realtime_pub = new realtime_tools::RealtimePublisher<myo_msgs::statusMessage>(n, "DebugMessage", 25);
    beforePos = joint_.getPosition();

    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // get excact time right now via clock_gettime

    b = no;
    clock_gettime(CLOCK_MONOTONIC,&no);
    ros::Time beforeMoment(b.tv_sec, b.tv_nsec);
    ros::Time thisMoment(no.tv_sec,no.tv_nsec);


    // get sensor values
    double position = joint_.getPosition();
    double velocity = joint_.getVelocity();
    double effort = joint_.getEffort();
    double displacement = joint_.getDisplacement();
    double analogIN0 = joint_.getAnalogIn(0);
    current = 0.8*current + 0.2*joint_.getEffort();

    // set pwm cycle [-4000;4000], here for safety reasons bound to [-500,500]
    // also calculate ref[RPM] to ref[Encodervalues/ms] : ~10240/60*1000 RPM = 1 ENC/ms
    double rpmref = (10240.0/60000.0)*ref;
    if(state == STARTUP){

      if(current > CURRENTMAX){
        ROS_INFO("currentbreak!");
        if(direction == CHANGED){
          minpos = (int)position;
          state = RUN;
          ROS_INFO("saved minpos %d",minpos);
        } else {
          direction = CHANGED;
          maxpos = (int)position;
          ROS_INFO("saved maxpos %d", maxpos);
        }
      }
      else{
          if(direction == UNCHANGED){
            joint_.setCommand(300);
          } else{
            joint_.setCommand(-300);
          }
        }
    }
    else{
      pwm = pwm -0.3*(velocity - rpmref);
      double pwmk = 0.05*rpmref + pwm;
      if(abs(pwmk) > 2000)
        pwmk = pwmk/abs(pwmk) *2000;
      joint_.setCommand(pwmk);
    };

    // realtime publish stuff (i.e. buffer and publish when ever possible)
    if (realtime_pub->trylock()){
      /*  float64  dt
          float64  position
          float64  velocity
          float64  velocity_ref
          float64  commanded_effort
          float64  analogIN0          */
      realtime_pub->msg_.position         = position;
      realtime_pub->msg_.velocity         = velocity;
      realtime_pub->msg_.velocity_ref     = rpmref;
      realtime_pub->msg_.analogIN0        = analogIN0;
      realtime_pub->msg_.dt               = thisMoment - beforeMoment;
      realtime_pub->msg_.current          = current;
      realtime_pub->msg_.commanded_effort = pwm;
      realtime_pub->unlockAndPublish();
    }
    beforePos = position;
  }

  // dummy stuff
  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

private:
  myo_interface::MyoMuscleJointHandle joint_;
};
}
#endif
