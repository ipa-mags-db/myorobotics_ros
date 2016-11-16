#ifndef __VELOCITYCONTROLLER_H
#define __VELOCITYCONTROLLER_H

#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>
// ROS Stuff
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>

// Messages
#include <myo_msgs/SetReference.h>
#include <myo_msgs/SetPID.h>
#include <myo_msgs/statusMessage.h>

#define MAXCUR 500
#define MAXREF 200
#define MAXPWM 2000


namespace myo_controllers{

class velocityController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
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
    double          err;
    double          errSum;
    struct timespec oldTime;
    double          oldPwm;
  } saveVals;

  struct PID
  {
    double  pgain;
    double  igain;
    double  dgain;
    double  ffgain;
    double  ffset;
  } pid;


  // reserve memory for Service & Topic Broadcaster
  ros::ServiceServer pidsrv_;
  ros::ServiceServer refsrv_;
  realtime_tools::RealtimePublisher<myo_msgs::statusMessage> *realtime_pub;



  template <typename T> int sgn(T val){
    return ((T(0) < val) - (val < T(0)));
  }


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

  bool setPID( myo_msgs::SetPID::Request& req,
               myo_msgs::SetPID::Response& resp)
  {
    if((req.pgain > -100) && (req.pgain <= 0) &&
       (req.igain > -100) && (req.igain <= 0) &&
       (req.dgain > -100) && (req.dgain <= 0) ){
      pid.pgain   = req.pgain;
      pid.igain   = req.igain;
      pid.dgain   = req.dgain;
      pid.ffgain  = req.ffgain;
      pid.ffset   = req.ffset;

      ROS_INFO("Change PID to P: %f, I: %f, D:%f, FFG: %f, FFS: %f",pid.pgain, pid.igain, pid.dgain, pid.ffgain, pid.ffset);
    } else {
        ROS_WARN("Reference set too high!");
        ROS_WARN("You tried to set to: P: %f, I: %f, D:%f, FFG: %f, FFS: %f",req.pgain, req.igain, req.dgain, req.ffgain, req.ffset);
      }
      resp.pgain  = pid.pgain;
      resp.igain  = pid.igain;
      resp.dgain  = pid.dgain;
      resp.ffgain = pid.ffgain;
      resp.ffset  = pid.ffset;
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
    saveVals.oldPwm = 0;
    saveVals.clutchState = true;
    saveVals.err = 0;
    saveVals.errSum = 0;
    pid.pgain = -10;
    pid.igain = 0;
    pid.dgain = -2;
    pid.ffgain = 0;
    pid.ffset  = 0;

    struct timespec no;
    clock_gettime(CLOCK_MONOTONIC,&no);
    saveVals.oldTime = no;

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure

    // Advertise Service and Publisher
    refsrv_ = n.advertiseService("set_reference", &velocityController::setReference, this);
    pidsrv_ = n.advertiseService("set_pid", &velocityController::setPID, this);
    realtime_pub = new realtime_tools::RealtimePublisher<myo_msgs::statusMessage>(n, "DebugMessage", 25);

    // Clutch on
    setClutch(false);

    // Exit graciously
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

    // filter current
    saveVals.filteredCurrent = 0.95*saveVals.filteredCurrent + 0.05*effort;


    //-------------------------
    //-- Velocity Controller --
    //-------------------------

    // parameter:

    saveVals.vel_ref = saveVals.ref;


    // Feedforward
    saveVals.pwm = pid.ffgain * saveVals.vel_ref + pid.ffset*sgn(saveVals.vel_ref);

    // Feedback P-Controller
      // calc error
      double err = velocity - saveVals.vel_ref;
      if(pid.igain != 0)
        saveVals.errSum += err;

      // P Gain
      saveVals.pwm += pid.pgain*err;

      // I Gain
      saveVals.pwm += pid.igain*saveVals.errSum;

      // D Gain
      saveVals.pwm += pid.dgain*(err - saveVals.err);

      // update oldError sum
      saveVals.err = err;

    // limit pwm to +/-3999
    if(fabs(saveVals.pwm) > MAXPWM)
      saveVals.pwm = sgn(saveVals.pwm) *MAXPWM;

    // anti-Windup
    saveVals.errSum += 10*(saveVals.oldPwm - saveVals.pwm);


    //------------------------
    //-- Compensate BackEMF --
    //------------------------
    double maxPWM = 2000 - 20*fabs(velocity);
    if(maxPWM < 200)
      maxPWM = 200;

    // Check if pwm change violates backEMF condition
    if(fabs(saveVals.pwm -saveVals.oldPwm) > maxPWM)
      saveVals.pwm = sgn(saveVals.pwm-saveVals.oldPwm)*maxPWM + saveVals.oldPwm;
    joint_.setCommand(saveVals.pwm);

    saveVals.oldPwm = saveVals.pwm;


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
      realtime_pub->msg_.position           = position;//position;
      realtime_pub->msg_.velocity           = velocity;
      realtime_pub->msg_.velocity_ref       = saveVals.vel_ref;
      realtime_pub->msg_.analogIN0          = analogIN0;
      realtime_pub->msg_.dt                 = dur;
      realtime_pub->msg_.commanded_effort   = saveVals.pwm;
      realtime_pub->msg_.displacement       = displacement;
      realtime_pub->msg_.displacement_ref   = saveVals.ref;
      realtime_pub->msg_.current            = saveVals.filteredCurrent;
      realtime_pub->msg_.clutchState        = saveVals.clutchState;
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
