#include "myo_controllers/displacementController.h"

namespace myo_controllers {

template <typename T> int sgn(T val) {
        return ((T(0) < val) - (val < T(0)));
}

bool displacementController::startup()
{
        ROS_INFO("Startup!");
        pC1.setPID(10,0,0);
        pC2.setPID(10,0,0);
        ff1.setPID(20,100,0);
        an1.setPID(2525,4096,0);
        count = 1;
        procVars.ref = 3;

        // Savety first!
        setDebugMode(0);
        setClutchState(1);

        return true;
}

void displacementController::loop()
{

        data.analogIN0 = an1.calcU(data.analogIN0);
        data.displacement = lp1.calcU(data.displacement);

        double disp_ref = procVars.ref;

        if(data.analogIN0 < -80) {
                if(count%10000 == 0) {
                        ROS_WARN("angle boundary violated, deducting %f disp_refs",0.1 * procVars.ref  *(data.analogIN0 + 80));
                }

                disp_ref += 0.03 * procVars.ref  *(data.analogIN0 + 80);
                if(disp_ref < 0)
                        disp_ref = 0;
        };
        double vel_ref = 0;

        vel_ref = pC2.calcU(disp_ref, data.displacement);

        if(count%10000 == 0) {
                ROS_INFO("disp_ref was %f",disp_ref);
                ROS_INFO("vel_ref was %f",vel_ref);
        }

        // make sure pwm is reset
        procVars.pwm = 0;

        // calculate FeedForward
        //procVars.pwm += ff1.calcU(vel_ref);

        if(count%10000 == 0) {
                ROS_INFO("pwm from FF was %d",procVars.pwm);
        }

        // calculate FeedBackward
        procVars.pwm += pC2.calcU(data.velocity,vel_ref);

        if(count%10000 == 0) {
                ROS_INFO("pwm from PID2 was %d",procVars.pwm);
        }

        if(abs(procVars.pwm) > 4000)
                procVars.pwm = myo_controllers::sgn(procVars.pwm)*4000;

        // limit PWM change
        procVars.pwm = emfChecker.calcU(procVars.pwm,data.velocity);

        if(count%10000 == 0) {
                count = 1;
        }
        count++;

}
}

PLUGINLIB_EXPORT_CLASS(myo_controllers::displacementController, controller_interface::ControllerBase);
