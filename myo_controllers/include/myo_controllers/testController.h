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
  int count;

  testController() : pC1("name1"), pC2('name2') {}
  void loop();
  bool startup();
};

}
#endif
