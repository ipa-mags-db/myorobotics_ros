#include "myo_controllers/testController.h"

namespace myo_controllers {
bool testController::startup()
{
        ROS_INFO("Startup!");
        pC1.setPID(10,0,0);
        count = 0;
        return true;
}

void testController::loop()
{

        procVars.pwm = pC1.calcU(procVars.ref,data.velocity);
        if(count%1000 == 0) {
                ROS_INFO("pwm was %d with reference value %f",procVars.pwm,procVars.ref);
                count = 0;
        }
        count++;
        //joint_.setCommand((double)pwm);

}
}

CLASS_LOADER_REGISTER_CLASS(myo_controllers::testController,myo_controllers::baseController);
PLUGINLIB_EXPORT_CLASS(myo_controllers::testController, controller_interface::ControllerBase);
