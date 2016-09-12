#ifndef __DISPLACEMENTCONTROLLER_H
#define __DISPLACEMENTCONTROLLER_H

#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>
// ROS Stuff
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

// Messages
#include <myo_msgs/SetReference.h>
#include <myo_msgs/statusMessage.h>

#define MAXCUR 200
#define MAXREF 200
#define MAXPWM 4000


namespace myo_controllers{

class displacementController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:

  // Struct to hold all values used more than one cycle
  struct SaveVals
  {
    double          pwm;
    double          ref;
    double          vel_ref;
    double          filteredCurrent;
    double          displacement_ref;
    bool            clutchState;
    struct timespec oldTime;
  } saveVals;


  // reserve memory for Service & Topic Broadcaster
  ros::ServiceServer srv_;
  realtime_tools::RealtimePublisher<myo_msgs::statusMessage> *realtime_pub;



  //----------------------
  //-- ROS Service call --
  //----------------------
  bool setReference( myo_msgs::SetReference::Request& req,
                    myo_msgs::SetReference::Response& resp)
  {
    if (fabs(req.reference) < MAXREF){
      saveVals.ref = req.reference;
      ROS_INFO("Change Reference to %f",saveVals.ref);
    } else
      ROS_WARN("Reference set too high! Max is +/-%d",MAXREF);

    resp.reference = saveVals.ref;
  }



  bool setClutch(bool input){
    joint_.setDigitalOut(input);
    ROS_INFO("Setting Clutch to %d",input);
  }



  bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    if (!n.getParam("joint", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // Init Values to 0
    saveVals.vel_ref = 0;
    saveVals.ref = 10;
    saveVals.filteredCurrent = 0;
    saveVals.pwm = 0;
    saveVals.clutchState = true;
    struct timespec no;
    clock_gettime(CLOCK_MONOTONIC,&no);
    saveVals.oldTime = no;

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure
      realtime_pub->msg_.displacement_ref   = saveVals.ref;

    // Advertise Service and Publisher
    srv_ = n.advertiseService("set_reference", &displacementController::setReference, this);
    realtime_pub = new realtime_tools::RealtimePublisher<myo_msgs::statusMessage>(n, "DebugMessage", 25);

    // Clutch on
    setClutch(true);

    // Exit graciously
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // TODO make hz meaningful
    int hz = 1000;

    // get sensor values
    double position = joint_.getPosition();
    double velocity = joint_.getVelocity();
    double effort = joint_.getEffort();
    double displacement = joint_.getDisplacement();
    double analogIN0 = joint_.getAnalogIn(0);


    // filter current
    saveVals.filteredCurrent = 0.95*saveVals.filteredCurrent + 0.05*effort;

    //-----------------------------
    //-- Displacement Controller --
    //-----------------------------
    saveVals.vel_ref =  -0.25*(saveVals.ref - displacement);


    //-------------------------
    //-- Velocity Controller --

    //-------------------------
    //-- Velocity Controller --
    saveVals.pwm += 0.05*saveVals.vel_ref;

    // set saveVals.pwm cycle [-4000;4000]
    if(abs(saveVals.pwm) > MAXPWM)
      saveVals.pwm = saveVals.pwm/abs(saveVals.pwm) *MAXPWM;
    joint_.setCommand(saveVals.pwm);


    //------------------
    //-- Calculate dt --
    //------------------
    struct timespec no;
    clock_gettime(CLOCK_MONOTONIC, &no);
    ros::Duration dur(no.tv_sec - saveVals.oldTime.tv_sec, no.tv_nsec - saveVals.oldTime.tv_nsec);
    saveVals.oldTime = no;

    //-------------------------
    //-- Current SavetyBreak --
    //-------------------------
    if (fabs(saveVals.filteredCurrent) > MAXCUR){
      ROS_WARN("Current was %f > %d", fabs(saveVals.filteredCurrent),MAXCUR);
      saveVals.clutchState = false;
      // Set Clutchvelocity_ref
      setClutch(saveVals.clutchState);
    }





    //-------------------
    //-- Publish stuff --
    //-------------------
    if (realtime_pub->trylock()){
      /*  float64  dt
          float64  position
          float64  velocity
          float64  velocity_ref
          float64  commanded_effort
          float64  analogIN0          */
      realtime_pub->msg_.position          = position;
      realtime_pub->msg_.velocity          = velocity;
      realtime_pub->msg_.velocity_ref      = saveVals.vel_ref;
      realtime_pub->msg_.analogIN0         = analogIN0;
      realtime_pub->msg_.dt                = dur;
      realtime_pub->msg_.commanded_effort  = effort;
      realtime_pub->msg_.displacement      = displacement;
      realtime_pub->msg_.displacement_ref  = saveVals.ref;
      realtime_pub->msg_.current           = saveVals.filteredCurrent;
      realtime_pub->msg_.clutchState       = saveVals.clutchState;
      realtime_pub->unlockAndPublish();
    }
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) {
    setClutch(false);
  }

private:
  myo_interface::MyoMuscleJointHandle joint_;
};
}
#endif
