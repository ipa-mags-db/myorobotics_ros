#ifndef __DISPLACEMENTCONTROLLER_H
#define __DISPLACEMENTCONTROLLER_H

#include <myo_controllers/baseController.h>

namespace myo_controllers {
class displacementController : public baseController
{
public:

  pidControl pC1;
  pidControl pC2;
  compensateEMF emfChecker;
  ffControl ff1;
  angleNorm an1;
  simpleLowPass lp1;

  int count;

  displacementController() : pC1("PID1"), pC2("PID2"), ff1("FF1"), emfChecker(2000,200,20), an1("AN1"), lp1(0.2)  {}
  void loop();
  bool startup();

};

}
#endif
