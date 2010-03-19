#include "cpp_laser.h"
#include "cpp_base.h"
#include "cpp_robot.h"
#include "cpp_simulator.h"
#include "cpp_carmenmap.h"
#include "cpp_mapdefinitions.h"

int main(int, char** ) {
  LaserMessage l;
  l.setTimestamp(carmen_get_time());

  OdometryMessage o;
  RobotLaserMessage r;
  TrueposMessage t;

  CarmenMap cm;
  RefProbMap rpm;
  FloatMap fm;
  DoubleMap dm;
  IntMap im;

  return 0;
}
