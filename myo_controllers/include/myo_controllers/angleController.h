#ifndef __angleController_H
#define __angleController_H

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

#define MAXCUR 1000
#define MAXREF 200
#define MAXPWM 4000


namespace myo_controllers{

  class angleController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
  {
  public:

    /*!
     * Function that implements signum for arbitrary
     * @tparam    T    Numeric datatype (e.g. double)
     * @param[in] val  Input for sign Function
     * @return    sign(@p val) of type @p T
     */
    template <typename T> int sgn(T val){
      return ((T(0) < val) - (val < T(0)));
    }


    /*!
     * Handling of ROS ServiceCalls that change Reference
     * @param[in]   req     Adress to request msg
     * @param[out]  resp    Adress to responose msg
     */
    void setReference( myo_msgs::SetReference::Request& req,
                      myo_msgs::SetReference::Response& resp)
    {
      if (fabs(req.reference) < MAXREF){
        saveVals_.ref = req.reference;
        ROS_INFO("Change Reference to %f",saveVals_.ref);
      } else
        ROS_WARN("Reference set too high! Max is +/-%d",MAXREF);

      resp.reference = saveVals_.ref;
    }



    /*!
     * Handling of ROS ServiceCalls that change PID values
     * @param[in]   req     Adress to request msg
     * @param[out]  resp    Adress to responose msg
     */
    void setPID( myo_msgs::SetPID::Request& req,
                 myo_msgs::SetPID::Response& resp)
    {
      if((req.pgain > -100) && (req.pgain <= 0) &&
         (req.igain > -100) && (req.igain <= 0) &&
         (req.dgain > -100) && (req.dgain <= 0) ){
        pid_.pgain   = req.pgain;
        pid_.igain   = req.igain;
        pid_.dgain   = req.dgain;
        pid_.ffgain  = req.ffgain;
        pid_.ffset   = req.ffset;

        ROS_INFO("Change PID to P: %f, I: %f, D:%f, FFG: %f, FFS: %f",pid_.pgain, pid_.igain, pid_.dgain, pid_.ffgain, pid_.ffset);
      } else {
          ROS_WARN("Reference set too high!");
          ROS_WARN("You tried to set to: P: %f, I: %f, D:%f, FFG: %f, FFS: %f",req.pgain, req.igain, req.dgain, req.ffgain, req.ffset);
        }
        resp.pgain  = pid_.pgain;
        resp.igain  = pid_.igain;
        resp.dgain  = pid_.dgain;
        resp.ffgain = pid_.ffgain;
        resp.ffset  = pid_.ffset;
    }

    /*!
     * Function that gets called before starting of the realtime loop
     * - used to setup all variables and ros nodes/topics/services
     * @param[in] hw      Pointer to hardwareInterface used
     * @param[in] n       Adress to ROS nodehandle
     */
    void setClutch(bool input){
      joint_.setDigitalOut(input);
      ROS_INFO("Setting Clutch to %d",input);
    }

    /*!
     * Function that gets called before starting of the realtime loop
     * - used to setup all variables and ros nodes/topics/services
     * @param[in]   hw      Pointer to hardwareInterface used
     * @param[in]   n       Adress to ROS nodehandle
     * @return      boolean if successful
     */
    bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n)
    {
      // get joint name from the parameter server
      std::string my_joint;
      if (!n.getParam("joint", my_joint)){
        ROS_ERROR("Could not find joint name");
        return false;
      }

      // Init Values to 0
      saveVals_.vel_ref = 0;
      saveVals_.ref = 10;
      saveVals_.filteredCurrent = 0;
      saveVals_.pwm = 0;
      saveVals_.clutchState = true;
      saveVals_.err = 0;
      saveVals_.errSum = 0;
      saveVals_.errSumDisp = 0;
      pid_.pgain = -10;
      pid_.igain = 0;
      pid_.dgain = -2;
      pid_.ffgain = 1;
      pid_.ffset  = 0;

      struct timespec no;
      clock_gettime(CLOCK_MONOTONIC,&no);
      saveVals_.oldTime = no;

      // get the joint object to use in the realtime loop
      joint_ = hw->getHandle(my_joint);  // throws on failure

      // Advertise Service and Publisher
      refsrv_ = n.advertiseService("set_reference", &angleController::setReference, this);
      pidsrv_ = n.advertiseService("set_pid", &angleController::setPID, this);
      realtime_pub_ = new realtime_tools::RealtimePublisher<myo_msgs::statusMessage>(n, "DebugMessage", 25);

      // Clutch on
      setClutch(true);

      // Exit graciously
      return true;
    }


    /*!
     * Function that gets called once in the control loop by the controller manager
     * - used to calculate all control algorithms
     * @param[in] time    Adress to current time stamp
     * @param[in] period  Periodic cycletime inbetween two calls of this function
     */
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
      saveVals_.filteredCurrent = 0.95*saveVals_.filteredCurrent + 0.05*effort;


      //-----------------------------
      //-- Displacement Controller --
      //-----------------------------
      double errdisp = (saveVals_.ref - displacement);
      saveVals_.errSumDisp += errdisp;

      saveVals_.vel_ref = -35*(saveVals_.ref - displacement) - 0 * saveVals_.errSumDisp;

      //-------------------------
      //-- Velocity Controller --
      //-------------------------


      // Feedforward
      saveVals_.pwm = pid_.ffgain * saveVals_.vel_ref + pid_.ffset * sgn(saveVals_.vel_ref);

      // Feedback P-Controller
        // calc error
        double err = velocity - saveVals_.vel_ref;
        if(pid_.igain != 0)
          saveVals_.errSum += err;

        // P Gain
        saveVals_.pwm += pid_.pgain*err;

        // I Gain
        saveVals_.pwm += pid_.igain*saveVals_.errSum;

        // D Gain
        saveVals_.pwm += pid_.dgain*(err - saveVals_.err);

        // update oldError sum
        saveVals_.err = err;




      double pwm_before = saveVals_.pwm;

      // limit pwm to +/-3999
      if(fabs(saveVals_.pwm) > MAXPWM)
        saveVals_.pwm = sgn(saveVals_.pwm) *MAXPWM;

      // anti-Windup
      saveVals_.errSum += 10*(pwm_before - saveVals_.pwm);
      //saveVals_.errSumDisp += 10*(pwm_before - saveVals_.pwm);

      joint_.setCommand(saveVals_.pwm);


      //------------------
      //-- Calculate dt --
      //------------------
      struct timespec no;
      clock_gettime(CLOCK_MONOTONIC, &no);
      ros::Duration dur(no.tv_sec - saveVals_.oldTime.tv_sec, no.tv_nsec - saveVals_.oldTime.tv_nsec);
      saveVals_.oldTime = no;

      //-------------------------
      //-- Current SavetyBreak --
      //-------------------------
      if (fabs(saveVals_.filteredCurrent) > MAXCUR){
        ROS_WARN("Current was %f > %d", fabs(saveVals_.filteredCurrent),MAXCUR);
        saveVals_.clutchState = false;
        // Set Clutchvelocity_ref
        setClutch(saveVals_.clutchState);
      }

      //-------------------
      //-- Publish stuff --
      //-------------------
      if (realtime_pub_->trylock()){
        /*  float64  dt
            float64  position
            float64  velocity
            float64  velocity_ref
            float64  commanded_effort
            float64  analogIN0          */
        realtime_pub_->msg_.position           = saveVals_.errSum;//position;
        realtime_pub_->msg_.velocity           = velocity;
        realtime_pub_->msg_.velocity_ref       = saveVals_.vel_ref;
        realtime_pub_->msg_.analogIN0          = analogIN0;
        realtime_pub_->msg_.dt                 = dur;
        realtime_pub_->msg_.commanded_effort   = saveVals_.pwm;
        realtime_pub_->msg_.displacement       = displacement;
        realtime_pub_->msg_.displacement_ref   = saveVals_.ref;
        realtime_pub_->msg_.current            = saveVals_.errSumDisp;
        realtime_pub_->msg_.clutchState        = saveVals_.clutchState;
        realtime_pub_->unlockAndPublish();
      }
    }

    void starting(const ros::Time& time) { }
    void stopping(const ros::Time& time) {
      setClutch(false);
    }

  private:

    myo_interface::MyoMuscleJointHandle joint_;

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
      double          errSumDisp;
      struct timespec oldTime;
    } saveVals_;

    struct PID
    {
      double  pgain;
      double  igain;
      double  dgain;
      double  ffgain;
      double  ffset;
    } pid_;

    // reserve memory for Service & Topic Broadcaster
    ros::ServiceServer pidsrv_;
    ros::ServiceServer refsrv_;
    realtime_tools::RealtimePublisher<myo_msgs::statusMessage> *realtime_pub_;

  };
}
#endif
