#ifndef __BASECONTROLLER_H
#define __BASECONTROLLER_H

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

namespace myo_controllers{

  class simpleLowPass
  {
  public:
    simpleLowPass(double alpha_in);
    double calcU(double input);
  private:
    double alpha;
    double oldVal;
  };

  class compensateEMF
  {
  public:
    compensateEMF(int max_in, int min_in, int slope_in);
    int calcU(int pwm,double velocity);
  private:
    int max;
    int min;
    int slope;
    int oldPWM;
  };

  class pidControl
  {
  public:

          pidControl(const char* inputName);
          int calcU(double ref, double out);
          bool setPID(double pval,double ival,double dval);
          static bool handleRX(std::string,double P,double I,double D);

  protected:

          static std::map<std::string,pidControl*> pidmap;

          // max pwm
          int maxpwm;
          int pwm;

          // Gains
          double pgain;
          double igain;
          double dgain;

          // Errors
          double errSum;
          double oldErr;
          double err;

  };

  class ffControl : public pidControl
  {
  public:
    ffControl(const char* inputName) : pidControl(inputName) {};
    int calcU(double ref);
  };

  class angleNorm : public pidControl
  {
  public:
    angleNorm(const char* inputName) : pidControl(inputName) {};
    double calcU(double ref);
  };


class baseController : public controller_interface::Controller<myo_interface::MyoMuscleJointInterface>
{
public:


        // reserve memory for Service & Topic Broadcaster
        ros::ServiceServer srvREF_;
        ros::ServiceServer srvPID_;
        ros::ServiceServer srvCLT_;

        // RealtimePublisher
        realtime_tools::RealtimePublisher<myo_msgs::statusMessage> *realtime_pub;

        struct sensorData{
          double position;
          double velocity;
          double effort;
          double displacement;
          double analogIN0;
          double current;
        }data;

        struct procVars{
          int     pwm;
          bool    clutchState;
          double  ref;
        }procVars;

        /**
         * @brief Mathematical operation sign
         * @param val input
         * @return sign(val)
         */
        template <typename T> int sgn(T val);

        /**
         * @brief A function calculate an angle from analog data
         * @param[in]  sensorVal analog sensor value (max 4096)
         * @return Angle in degree
         */
        double calcAngle(double sensorVal);

        /**
         * @brief A function to handle ROS Service calls to set Reference
         * @param[in]  req variable to hold requested reference change
         * @param[out] resp new reference value
         * @return successful or not
         */
        bool setReference( myo_msgs::SetReference::Request& req, myo_msgs::SetReference::Response& resp);


        /**
         * @brief A function to handle ROS Service calls to set PID values
         * @param[in] req variable to hold requested PID and FF gains change
         * @param[out] resp new PID and FF gains
         * @return successful or not
         */
        bool setPID( myo_msgs::SetPID::Request& req, myo_msgs::SetPID::Response& resp);

        void setDebugMode(bool mode);

        bool CLTRxHandle( myo_msgs::SetClutch::Request& req, myo_msgs::SetClutch::Response& resp);

        /**
         * @brief A function to (de)couple clutch
         * @param[in] input New state for Clutch
         * @return successful or not
         * @TODO actually evaluate if setting clutch worked (i.e. dont always return true)
         */
        bool setClutchState(bool input);

        //**************
        // Needed to satisfy ROS_Controller Hardwareinterface
        //**************

        bool init(myo_interface::MyoMuscleJointInterface* hw, ros::NodeHandle &n);

        void update(const ros::Time& time,const ros::Duration& period);

        void stopping(const ros::Time& time);

        void starting(const ros::Time& time);

        //**************

        bool getData();
        virtual void loop() = 0;
        virtual bool startup() = 0;
        void publish();
        // Handle to ROS-control Joint
        myo_interface::MyoMuscleJointHandle joint_;

protected:

        bool debugMode;

private:


        // absolute Time of last loop start
        struct timespec no;

        // counter for braking time
        int counter;
};

}
#endif
