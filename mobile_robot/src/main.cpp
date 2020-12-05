#include <iostream>
#include <math.h>
#include <cmath>
#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  envir.addRobot(robot);
  robot.initWheels(0.07,0.3,10);
  RangeSensor rangeSensor(robot,0.1,0,0);


  Robot robot2("bb8", 0,0,0);
  envir.addRobot(robot2);
  robot2.initWheels(0.05, 0.3, 10);
 BearingSensor bearingsensor(robot2, 0.1, 0, 0);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());
    robot2.goTo(envir.target());

  }

  // plot trajectory
  envir.plot();

}
