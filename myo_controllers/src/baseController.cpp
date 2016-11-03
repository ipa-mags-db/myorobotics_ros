#include "myo_controllers/baseController.h"

namespace myo_controllers {

//-----------------------------
//-- PID Controller -----------
//-----------------------------

std::map<std::string, pidControl *> pidControl::pidmap;

pidControl::pidControl(const char* inputName) {
        std::string name(inputName);
        pidControl::pidmap.insert(std::pair<std::string, pidControl *>(name, this));
        ROS_INFO("Insert %s to pidmap",name.c_str());
        maxpwm = 4000;
        pwm = 0;

        pgain = 0;
        igain = 0;
        dgain = 0;

        errSum = 0;
        err = 0;
        oldErr = 0;
};

bool pidControl::setPID(double pval, double ival, double dval) {
        pgain = pval;
        igain = ival;
        dgain = dval;
        return true;
};

int pidControl::calcU(double ref, double out) {
        // Calculate Error
        err = out - ref;

        // Only update errorsum if igain is set
        if (fabs(igain) > 0.001)
                errSum += err;

        // P Gain
        pwm = pgain * err;

        // I Gain
        pwm += igain * errSum;

        // D Gain
        pwm += dgain * (err - oldErr);

        // update oldError sum
        oldErr = err;

        // check if pwm is in bound
        if (pwm > maxpwm)
                pwm = maxpwm;
        else if (pwm < -maxpwm)
                pwm = -maxpwm;

        return pwm;
};

bool pidControl::handleRX(std::string name, double P, double I, double D) {
        ROS_INFO("Checking Sub_controller map...");
        for(std::map<std::string, pidControl *>::const_iterator it = pidControl::pidmap.begin();
            it != pidControl::pidmap.end(); ++it)
        {
                ROS_INFO("Name: %s \n",it->first.c_str());
        }
        if (pidControl::pidmap.find(name) != pidControl::pidmap.end()) {
                pidControl::pidmap.find(name)->second->setPID(P, I, D);
                return true;
        } else {
                return false;
        }
};

//----------------------------------
//-- base Class for Controllers ----
//----------------------------------

double baseController::calcAngle(double sensorVal) {
        double maxSensor = 4096;
        double setPoint = 2525;
        return 360 * (sensorVal - setPoint) / maxSensor;
};

bool baseController::setReference(myo_msgs::SetReference::Request &req,
                                  myo_msgs::SetReference::Response &resp) {

        if(fabs(req.reference) > 4000) {
                ROS_WARN("Reference set too high! Please choose a value in range [-4000,4000]");
                resp.reference = procVars.ref;
                return false;
        }
        else{
                procVars.ref = req.reference;
                resp.reference = procVars.ref;
                ROS_INFO("Setting Reference to %f",req.reference);
                return true;
        }

};

bool baseController::setPID(myo_msgs::SetPID::Request &req,
                            myo_msgs::SetPID::Response &resp) {

        ROS_INFO("Trying to set %s to [P:%f|I:%f|D:%f] ... ",req.name.c_str(),req.pgain,req.igain,req.dgain);
        bool state = pidControl::handleRX(req.name.c_str(),req.pgain,req.igain,req.dgain);
        if(state == true)
                ROS_INFO("Sucess!");
        else
                ROS_INFO("Fail...");
        resp.pgain  = req.pgain;
        resp.igain  = req.igain;
        resp.dgain  = req.dgain;
        return state;
};

bool baseController::setClutch(bool input) {
        joint_.setDigitalOut(input);
        ROS_INFO("Setting Clutch to %d", input);
        return true;
};

bool baseController::init(myo_interface::MyoMuscleJointInterface *hw,
                          ros::NodeHandle &n) {
        // get joint name from the parameter server
        std::string my_joint;
        if (!n.getParam("joint", my_joint)) {
                ROS_ERROR("Could not find joint name");
                return false;
        }

        // Init timestamp
        clock_gettime(CLOCK_MONOTONIC, &this->no);

        // get the joint object to use in the realtime loop
        joint_ = hw->getHandle(my_joint); // throws on failure

        procVars.pwm = 0;
        procVars.clutchState = false;
        procVars.ref = 0;

        getData();

        // Advertise Service and Publisher
        srvREF_ = n.advertiseService("set_reference", &baseController::setReference, this);
        srvPID_ = n.advertiseService("set_controll", &baseController::setPID, this);

        realtime_pub = new realtime_tools::RealtimePublisher<myo_msgs::statusMessage>(
                n, "DebugMessage", 25);

        return startup();
};

bool baseController::getData(){
        data.position       = joint_.getPosition();
        data.velocity       = joint_.getVelocity();
        data.effort         = joint_.getEffort();
        data.displacement   = joint_.getDisplacement();
        data.current        = 0.9*joint_.getEffort(); +0.1*data.current;
        data.analogIN0      = joint_.getAnalogIn(0);
}

bool baseController::startup(){
        return true;
}

void baseController::loop(){
        //ROS_INFO("replace this loop");
}

void baseController::publish(){
        if (realtime_pub->trylock()) {
                /*  time     time
                    float64  position
                    float64  velocity
                    float64  displacement
                    float64  reference
                    float64  current
                    float64  commanded_effort
                    float64  analogIN0
                    int32    clutchState
                 */
                ros::Time t1(no.tv_sec, no.tv_nsec);

                realtime_pub->msg_.position           = data.position;
                realtime_pub->msg_.velocity           = data.velocity;
                realtime_pub->msg_.analogIN0          = data.analogIN0;
                realtime_pub->msg_.displacement       = data.displacement;
                realtime_pub->msg_.current            = data.current;

                realtime_pub->msg_.time                 = t1;
                realtime_pub->msg_.commanded_effort   = procVars.pwm;

                realtime_pub->msg_.reference          = procVars.ref;

                realtime_pub->msg_.clutchState        = procVars.clutchState;

                realtime_pub->unlockAndPublish();
        }
}

void baseController::update(const ros::Time& time, const ros::Duration& period){
        clock_gettime(CLOCK_MONOTONIC, &this->no);
        getData();
        loop();
        publish();
}

void baseController::stopping(const ros::Time& time){

}

void baseController::starting(const ros::Time& time){

}

//----------------------
//-- Helper functions --
//----------------------

template <typename T> int sgn(T val) {
        return ((T(0) < val) - (val < T(0)));
}

}
