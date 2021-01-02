// Answers to the questions:
// Part 1
// Question 1: In which file is the target motion defined?
// Answer 1: The target motion is defined in the envir.cpp file (cardiod curve)
// and is called within the main.cpp.

// Question 2: Explain the signature of Robot::Robot, especially the way to pass arguments.
// Answer 2: By using Robot::Robot, we instantiate a new instance of the Robot class. With the arguments
// we pass while instantiating, we define the name and the initial values of x, y and theta. The values
// of x,y get pushed into lists x_history and y_history.

// Question 3: Implement the function Robot::moveVW
// Answer 3: This function is implemented in robot.cpp

// Question 4: Now that a realistic way to control the robot is possible, should the Robot::moveXYT
// method stay available for external use? What can we do in the robot.h file to make it impossible to
// use it from outside the Robot class?
// Answer 4: Instead of defining it as public, I defined this method as protected and thereby made it impossible
// to be used outside the Robot class.

// Question 5-1: Modify moveWithSensor to use moveVW
// Answer 5-1: The modified code is in robot.cpp

// Question 5-2: Define new attributes and implement initWheel method
// Answer 5-2: The new attributes and the new initWheel method were added and implemented
// in robot.h and robot.cpp

// Question 6: Implement the Robot::rotateWheels method, so that it can be possible to control the
// robot by sending wheel velocities.
// Answer 6: Implemented in robot.cpp

// Question 7: By using a bool wheels init attribute, make sure that it is impossible to do anything
// in Robot::rotateWheels if the radius and base have not been initialized.
// Answer 7: The boolean attribute wheels_init is set to True only and only once the radius and base have
// been initialized.

// Part 2: Velocity Limits
// Question 1: Modify the Robot::initWheels method in order to pass a new argument that defines
// the wheel angular velocity limit. You may need to define a new attribute of the Robot
// class to store this limit.
// Answer 1: This was implemented in robot.h and robot.cpp

// Question 2: Modify the Robot::rotateWheels method in order to ensure that the applied velocities
// (ω l , ω r ) are within the bounds.
// Answer 2: The answer to this question was implemented in robot.cpp. First we check the scaling
// factor. If it's below 1, it is then set to 1, otherwise it remains the same. Afterwards, the velocities
// are scaled using that scaling factor.

// Question 3: Modify the Robot::moveVW method so that a (v, ω)
// setpoint is changed to a (ω l , ω r ) setpoint that will then be called through Robot::rotateWheels.
// Answer 3: This is implemented in robot.cpp

// The answers to questions from Part 3:Range Sensor and Part 4: Bearing Sensor were given inside the code.



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
  // radius and base of the first robot
  double r1 = 0.07;
  double b1 = 0.3;
  double omega_limit1 = 10;
  robot.initWheel(r1, b1, omega_limit1);
  RangeSensor range_sensor1 (robot, 0.1, 0, 0);
  //RangeSensor range_sensor2 (robot, 0.3, 0.0, 0.0);



  Robot robot2("Robocop", 0, 0, 0);
  double r2 = 0.07;
  double b2 = 0.05;
  double omega_limit2 = 10;
  robot2.initWheel(r2, b2, omega_limit2);
  BearingSensor bearing_sensor1 (robot2, 0.1, 0, 0);

  envir.addRobot(robot2);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());

    robot2.moveWithSensor(Twist(0.4, 0, 0));
    cout << endl;
    robot.printPosition();
    robot2.printPosition();

  }

  // plot trajectory
  envir.plot();

}
