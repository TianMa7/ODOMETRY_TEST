#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

// START IQ MACROS
#define waitUntil(condition) \
  do                         \
  {                          \
    wait(5, msec);           \
  } while (!(condition))

#define repeat(iterations) \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS

// Robot configuration code.
inertial BrainInertial = inertial();
motor MotorLeft = motor(PORT11, false);
motor MotorRight = motor(PORT8, true);
motor MotorIntake = motor(PORT10, false);

// generating and setting random seed
void initializeRandomSeed()
{
  wait(100, msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
      xAxis + yAxis + zAxis);
  // Set the seed
  srand(seed);
}

// Converts a color to a string
const char *convertColorToString(color col)
{
  if (col == colorType::red)
    return "red";
  else if (col == colorType::green)
    return "green";
  else if (col == colorType::blue)
    return "blue";
  else if (col == colorType::white)
    return "white";
  else if (col == colorType::yellow)
    return "yellow";
  else if (col == colorType::orange)
    return "orange";
  else if (col == colorType::purple)
    return "purple";
  else if (col == colorType::cyan)
    return "cyan";
  else if (col == colorType::black)
    return "black";
  else if (col == colorType::transparent)
    return "transparent";
  else if (col == colorType::red_violet)
    return "red_violet";
  else if (col == colorType::violet)
    return "violet";
  else if (col == colorType::blue_violet)
    return "blue_violet";
  else if (col == colorType::blue_green)
    return "blue_green";
  else if (col == colorType::yellow_green)
    return "yellow_green";
  else if (col == colorType::yellow_orange)
    return "yellow_orange";
  else if (col == colorType::red_orange)
    return "red_orange";
  else if (col == colorType::none)
    return "none";
  else
    return "unknown";
}

// Convert colorType to string
const char *convertColorToString(colorType col)
{
  if (col == colorType::red)
    return "red";
  else if (col == colorType::green)
    return "green";
  else if (col == colorType::blue)
    return "blue";
  else if (col == colorType::white)
    return "white";
  else if (col == colorType::yellow)
    return "yellow";
  else if (col == colorType::orange)
    return "orange";
  else if (col == colorType::purple)
    return "purple";
  else if (col == colorType::cyan)
    return "cyan";
  else if (col == colorType::black)
    return "black";
  else if (col == colorType::transparent)
    return "transparent";
  else if (col == colorType::red_violet)
    return "red_violet";
  else if (col == colorType::violet)
    return "violet";
  else if (col == colorType::blue_violet)
    return "blue_violet";
  else if (col == colorType::blue_green)
    return "blue_green";
  else if (col == colorType::yellow_green)
    return "yellow_green";
  else if (col == colorType::yellow_orange)
    return "yellow_orange";
  else if (col == colorType::red_orange)
    return "red_orange";
  else if (col == colorType::none)
    return "none";
  else
    return "unknown";
}

void vexcodeInit()
{

  // Initializing random seed.
  initializeRandomSeed();
}

#pragma endregion VEXcode Generated Robot Configuration

//----------------------------------------------------------------------------
//
//    Module:       main.cpp
//    Author:       {Tian Ma}
//    Created:      {28 October 2025}
//    Description:  Odometry test code for card dealer
//
//----------------------------------------------------------------------------

// Include the IQ Library
#include "iq_cpp.h"
#include "movement.h"

// Allows for easier use of the VEX Library
using namespace vex;

int main()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code
  Robot cardin(MotorLeft, MotorRight, Brain, BrainInertial);

  cardin.moveTo(100,-100, 90, 50);

  Brain.Screen.print("X:%.2f, Y: %.2f\n", cardin.getX(), cardin.getY());
}

void Robot::moveTo(float centerX, float centerY, float finalAngle, float speed = 100)
{
  static const int lastRadius = 30; // cm radius of arc to turn
  static int lastSpeed = speed;         // int used to smooth speed
  const int error = 5;              // acceptable movement error for robot
  float deltaAngle = normalizeAngle(BrainInertial.heading(), findTangent(centerX, centerY, lastRadius));
  float speedRatio = arcRatio(lastRadius);

  // Brain.Screen.print("%.2f\n", speedRatio);
  // Brain.Screen.print("PHASE1\n");
  // Brain.Screen.print("%.2f\n", deltaAngle);
  // Brain.Screen.newLine();

  if (deltaAngle < 0) // needs to turn ccw
  {
    MotorLeft.setVelocity(speedRatio * lastSpeed, percent);
    MotorRight.setVelocity(lastSpeed, percent);
    Brain.Screen.print("L");
  }
  else // needs to turn cw
  {
    MotorLeft.setVelocity(lastSpeed, percent);
    MotorRight.setVelocity(lastSpeed * speedRatio, percent);
    Brain.Screen.print("R");
  }

  MotorLeft.spin(forward);
  MotorRight.spin(forward);

  // Wait until we are tanget to circle
  while (abs(deltaAngle) >  5)
  {
    // // slow accelerate function
    // if (lastSpeed == speed)
    // {
    //   // skip the rest of the checks
    // }
    // else if (lastSpeed < speed)
    // {
    //   lastSpeed += 5;
    // }
    // else
    // {
    //   lastSpeed = speed;
    // }

    if (deltaAngle < 0) // needs to turn ccw
    {
      MotorLeft.setVelocity(speedRatio * lastSpeed, percent);
      MotorRight.setVelocity(lastSpeed, percent);
    }
    else // needs to turn cw
    {
      MotorLeft.setVelocity(lastSpeed, percent);
      MotorRight.setVelocity(lastSpeed * speedRatio, percent);
    }

    // updates odometry and find a new tangent based on updated location
    locationUpdate();
    deltaAngle = normalizeAngle(BrainInertial.heading(), findTangent(centerX, centerY, lastRadius));
  }

  MotorLeft.setVelocity(lastSpeed, percent);
  MotorRight.setVelocity(lastSpeed, percent);
  // MIGHT WANT TO ADD A SMOOTH INCREASE ON SPEED HERE
  Brain.Screen.print("phase3\n");
  Brain.Screen.print("%.2f\n", deltaAngle);
  Brain.Screen.newLine();
  // Goes straight

  // Waits until we hit the tangent
  Brain.Screen.print("X:%.2f, Y: %.2f\n", location[0], location[1]);
  Brain.Screen.newLine();
  while (hypot((location[1] - centerY), (location[0] - centerX)) > (5 + lastRadius) )
  {
    if (abs(deltaAngle) < error) // on course
    {
      MotorLeft.setVelocity(lastSpeed, percent);
      MotorRight.setVelocity(lastSpeed, percent);
    }
    else if(deltaAngle < 0)//ccw drift
    {
      MotorLeft.setVelocity(lastSpeed - 3, percent);
    }
    else //cw drift
    {
      MotorRight.setVelocity(lastSpeed - 3, percent);
    }

    //update odom
    locationUpdate();
    deltaAngle = normalizeAngle(BrainInertial.heading(), findTangent(centerX, centerY, lastRadius));
  }

  Brain.Screen.print("phase4\n");
  // Enters final arc to drive in
  MotorLeft.setVelocity(lastSpeed, percent);
  MotorRight.setVelocity(lastSpeed * speedRatio, percent);
  while (abs(BrainInertial.heading() - finalAngle) > 5)
  {
    // reduce speed as we get close, should be almost no speed once we get there
    // smooth distrubution should start here
    locationUpdate();
  }

  // stops motors to ensure that odometry remains accurate)
  //DELETE THESE IF WE ARE NOT STOPPING AFTER MOVE TO

  MotorRight.stop();
  MotorLeft.stop();
}

void Robot::locationUpdate()
{
  double waitTime = 10; // ms (ADJUST AS NEEDED)

  // Initial Values
  double lastLeft = MotorLeft.position(turns);
  double lastRight = MotorRight.position(turns);
  double lastCenter = (lastLeft + lastRight) / 2.0;

  wait(waitTime, msec); // wait before starting updates

  // Final Values
  double currentLeft = MotorLeft.position(turns);
  double currentRight = MotorRight.position(turns);
  double currentCenter = (currentLeft + currentRight) / 2.0;
  double currentHeading = BrainInertial.heading(degrees) * (M_PI / 180.0);

  // Find change in position
  // Update wheelCircumference to more precise results
  double deltaCenter = (currentCenter - lastCenter) * wheelCircumference;

  // Updates values of x and y
  location[0] += deltaCenter * cos(currentHeading); // x
  location[1] += deltaCenter * sin(currentHeading); // y
}

// DRIVETRAIN PRIVATE FUNCTIONS:
void Robot::configureAllSensors()
{
  BrainInertial.calibrate();
  wait(2, seconds);
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  MotorLeft.setPosition(0, turns);
  MotorRight.setPosition(0, turns);
  // Brain.Screen.clearScreen();
  // Brain.Screen.setFont(mono15);
  location[0] = 0;
  location[1] = 0;

  // might want to add a wait here to ensure calibration is done
}


//steps:
/*
1. take all nodes(x, y, angle)
2. find centroid of all nodes
3. find the vector from centroid to each node
4. find the center of each node
5. output info into an array

*/

void findCentroid(float nodeX[], float nodeY[], float numNodes)
{
  float centerX = 0;
  float centerY = 0;
  if(numNodes < 3)
  {
    centerX = (nodeX[0] + nodeX[1])/2;
    centerY = (nodeY[0] + nodeY[1])/2;
  }
  else
  {
    float area = 0;
  }
}