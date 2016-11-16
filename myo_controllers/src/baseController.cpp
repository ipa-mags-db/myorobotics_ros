#include "myo_controllers/baseController.h"

#define PWMLIMIT 4000

namespace myo_controllers {


//----------------------
//-- Helper functions --
//----------------------

template <typename T> int sgn(T val) {
        return ((T(0) < val) - (val < T(0)));
}



simpleLowPass::simpleLowPass(double alpha_in){
        if(alpha_in > 0 && alpha_in < 1)
                alpha = alpha_in;
        else
                alpha = 0.5;
        oldVal = 0;
}
double simpleLowPass::calcU(double input){
        oldVal =  alpha*input + (1-alpha)*oldVal;
        return oldVal;
}




//-----------------------------
//--- Compansate backEMF ------
//-----------------------------
compensateEMF::compensateEMF(int max_in, int min_in, int slope_in){
        compensateEMF::max = max_in;
        compensateEMF::min = min_in;
        compensateEMF::slope = slope_in;
        oldPWM = 0;
        ROS_INFO("setting to max %d, min %d, slope %d",max,min,slope);

}

int compensateEMF::calcU(int pwm, double velocity){
        int maxPWM = max - slope*(int)fabs(velocity);
        if(maxPWM < min)
                maxPWM = min;
        // Check if pwm change violates backEMF condition
        if(abs(pwm - oldPWM) > maxPWM) {
                //  ROS_INFO("signum: %d , maxPWM: , oldPWM ")
                pwm = myo_controllers::sgn(pwm-oldPWM)*maxPWM + oldPWM;
        }
        oldPWM = pwm;
        return pwm;
};

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



int ffControl::calcU(double ref) {
        int pwm = (int)(pgain * ref);
        pwm += (int)igain*sgn(ref);
        if(abs(pwm) > maxpwm)
                pwm = sgn(pwm)*maxpwm;
        return pwm;
}


double angleNorm::calcU(double ref) {
        double maxSensor = igain;
        double setPoint  = pgain;
        return (360*(ref-setPoint)/maxSensor);
}



//----------------------------------
//-- base Class for Controllers ----
//----------------------------------

void baseController::setDebugMode(bool mode){
        debugMode = mode;
};


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
bool baseController::CLTRxHandle( myo_msgs::SetClutch::Request& req,
                                  myo_msgs::SetClutch::Response& resp)
{
        bool nState = req.clutchState;
        setClutchState(nState);
        resp.clutchState = procVars.clutchState;
}

bool baseController::setClutchState(bool input) {
        procVars.clutchState = input;
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
        counter = 0;
        debugMode = 1;

        getData();

        // Advertise Service and Publisher
        srvREF_ = n.advertiseService("set_ref", &baseController::setReference, this);
        srvPID_ = n.advertiseService("set_ctr", &baseController::setPID, this);
        srvCLT_ = n.advertiseService("set_clt", &baseController::CLTRxHandle, this);

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
        if (counter == 0)
        {
                if(abs(procVars.pwm) > PWMLIMIT) {
                        ROS_WARN("PWM greater than %d (= %f)! Braking Motor!",PWMLIMIT, 100.0*(double)PWMLIMIT/4000.0);
                        procVars.pwm = 0;
                        counter = 1000;
                };
        }
        else{
                procVars.pwm = 0;
                counter--;
        }
        if(debugMode == 0)
                joint_.setCommand((double)procVars.pwm);
        publish();
}

void baseController::stopping(const ros::Time& time){

}

void baseController::starting(const ros::Time& time){

}

}
