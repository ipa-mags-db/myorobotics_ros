#ifndef __TESTCONTROLLER_H
#define __TESTCONTROLLER_H

/*
#include <controller_interface/controller.h>
#include <myo_interface/myoMuscleJointInterface.h>
#include <pluginlib/class_list_macros.h>
*/
#include <myo_controllers/baseController.h>

namespace myo_controllers {
class testController : public baseController
{
public:

  pidControl pC1;
  pidControl pC2;
  compensateEMF emfChecker;
  ffControl ff1;
  angleNorm an1;
  simpleLowPass lp1;

  int count;

  testController() : pC1("PID1"), pC2("PID2"), ff1("FF1"), emfChecker(2000,200,20), an1("AN1"), lp1(0.2)  {}
  void loop();
  bool startup();

};

}
#endif
