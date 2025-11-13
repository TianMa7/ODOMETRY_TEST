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
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END IQ MACROS


// Robot configuration code.
inertial BrainInertial = inertial();
motor MotorLeft = motor(PORT11, false);
motor MotorRight = motor(PORT8, true);
motor MotorIntake = motor(PORT10, false);

// generating and setting random seed
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  // Combine these values into a single integer
  int seed = int(
    xAxis + yAxis + zAxis
  );
  // Set the seed
  srand(seed); 
}

// Converts a color to a string
const char* convertColorToString(color col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}


// Convert colorType to string
const char* convertColorToString(colorType col) {
  if (col == colorType::red) return "red";
  else if (col == colorType::green) return "green";
  else if (col == colorType::blue) return "blue";
  else if (col == colorType::white) return "white";
  else if (col == colorType::yellow) return "yellow";
  else if (col == colorType::orange) return "orange";
  else if (col == colorType::purple) return "purple";
  else if (col == colorType::cyan) return "cyan";
  else if (col == colorType::black) return "black";
  else if (col == colorType::transparent) return "transparent";
  else if (col == colorType::red_violet) return "red_violet";
  else if (col == colorType::violet) return "violet";
  else if (col == colorType::blue_violet) return "blue_violet";
  else if (col == colorType::blue_green) return "blue_green";
  else if (col == colorType::yellow_green) return "yellow_green";
  else if (col == colorType::yellow_orange) return "yellow_orange";
  else if (col == colorType::red_orange) return "red_orange";
  else if (col == colorType::none) return "none";
  else return "unknown";
}


void vexcodeInit() {

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

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // Begin project code
  Robot cardin(MotorLeft, MotorRight, Brain, BrainInertial);

  cardin.driveTest();
  // cardin.startLocationThread();
  // MotorIntake.spin(reverse, 50, percent);

  Brain.Screen.print("X:%.2f, Y: %.2f\n", cardin.getX(), cardin.getY());
}

//DRIVETRAIN PUBLIC FUNCTIONS:
    void Robot::driveTest()
    {
        driveStraight(100, 50);
    }

    void Robot::moveTo(float x, float y, float finalAngle, float speed = 100)
    {
      static const int lastRadius = 10; //cm
      const int error = 0;
      float deltaAngle = tangentFunction(target);
      float speedRatio = arcRatio(lastRadius);

      if(deltaAngle < 0)//needs to turn ccw
      {
        MotorLeft.setVelocity(speedRatio * speed, percent);
        MotorRight.setVelocity(speed, percent);
      }
      MotorLeft.spin(forward);
      MotorRight.spin(forward);
      while(deltaAngle != (0 + error))
      {
        //set a function to reduce the speed of speed as we approach
      }
      
      float currentAngle = BrainInertial.heading(degrees) * M_PI / 180;

      //while (currentAngle + finalAngle - 2arctan(deltaY/deltaX != 0)
      //ADD A BUFFER ZONE(ie. +- 1 deg to this while loop)
      //CHECK FOR SECOND CASE WHERE WE APPROACH 0 FROM OTHER SIDE

      //rotate in a small radius arc, switch depending on what direction is faster to finish loop
      while(currentAngle + finalAngle - (2*atan2f(y - location[1], x - location[0])) != 0)
      {
        //implement arc PID, slow down overall speed as we approach 0
      }

      //Calculate arc
      //Do math to find radius with x, y, finalAngle and location[0], location[1], heading
      //do math to find the angle between p1 and p2
      //drive in an arc with (radius, angle, speed)
    }

//DRIVETRAIN PRIVATE FUNCTIONS:
void Robot::configureAllSensors()
    {
        BrainInertial.calibrate();
        wait(2, seconds);
        BrainInertial.setHeading(0, degrees);
        BrainInertial.setRotation(0, degrees);
        LeftMotor.setPosition(0, turns);
        RightMotor.setPosition(0, turns);
        // Brain.Screen.clearScreen();
        // Brain.Screen.setFont(mono15);
        location[0] = 0;
        location[1] = 0;

        // might want to add a wait here to ensure calibration is done
    }

    void Robot::locationUpdate()
    {
        double waitTime = 10; // ms (ADJUST AS NEEDED)

        // Initial Values
        double lastLeft = LeftMotor.position(turns);
        double lastRight = RightMotor.position(turns);
        double lastCenter = (lastLeft + lastRight) / 2.0;

        wait(waitTime, msec); // wait before starting updates

        // Final Values
        double currentLeft = LeftMotor.position(turns);
        double currentRight = RightMotor.position(turns);
        double currentCenter = (currentLeft + currentRight) / 2.0;
        double currentHeading = BrainInertial.heading(degrees) * (M_PI / 180.0);

        // Find change in position
        // Update wheelCircumference to more precise results
        double deltaCenter = (currentCenter - lastCenter) * wheelCircumference;

        // Updates values of x and y
        location[0] += deltaCenter * cos(currentHeading); // x
        location[1] += deltaCenter * sin(currentHeading); // y
    }

    void Robot::driveArc(float radius, float angle, float speed)
    {
        // Implement driving in an arc with given radius and angle
        // This is a placeholder for actual arc driving logic
        printf("Driving arc with radius %.2f, angle %.2f at speed %.2f\n", radius, angle, speed);
    }

    // UNSURE IF NEEDED
    float Robot::trackingCenter()
    {
        float leftDistance = LeftMotor.position(turns);
        float rightDistance = RightMotor.position(turns);
        float centerPosition = (leftDistance + rightDistance) / 2.0;
        return centerPosition;
    }

    // void Robot::driveStraight(float distance, float maxSpeed)
    // {
    //     float initialPosition = LeftMotor.position(turns) * wheelCircumference;
    //     float initialRotation = BrainInertial.rotation(degrees);
    //     float distanceRemaining = distance;
    //     float targetPosition = initialPosition + distance;
    //     LeftMotor.spin(forward);
    //     RightMotor.spin(forward);
    //     while ((trackingCenter() - initialPosition) * wheelCircumference < distance)
    //     {
    //         distanceRemaining = targetPosition - (trackingCenter() * wheelCircumference);
    //         LeftMotor.setVelocity(pLeft(distanceRemaining, maxSpeed, initialRotation), percent);
    //         RightMotor.setVelocity(pRight(distanceRemaining, maxSpeed, initialRotation), percent);
    //         locationUpdate();
    //     }
    //     LeftMotor.stop();
    //     RightMotor.stop();
    //     printf("Driving straight for distance %.2f at max speed %.2f\n", distance, maxSpeed);
    // }

    float Robot::pLeft(float distanceRemaining, float maxSpeed = 100, float initialRotation = NULL)
    {
        // Simple proportional controller
        float a = 10.0;
        float b = 5.0;
        float percentReturn = 100;
        percentReturn = ((distanceRemaining) / (distanceRemaining + a)) * (maxSpeed - b) + b;
        // equation
        float headingError = BrainInertial.rotation(degrees) - initialRotation;
        if (headingError > 0 || initialRotation != NULL)
        {
            percentReturn -= headingError; // Adjust the factor as needed
        }
        return percentReturn;
    }

    float Robot::pRight(float distanceRemaining, float maxSpeed = 100, float initialRotation = NULL)
    {
        // Simple proportional controller
        float a = 10.0;
        float b = 5.0;
        float percentReturn = 100;
        percentReturn = ((distanceRemaining) / (distanceRemaining + a)) * (maxSpeed - b) + b;
        // equation
        float headingError = BrainInertial.rotation(degrees) - initialRotation;
        if (headingError < 0 || initialRotation != NULL)
        {
            percentReturn -= -headingError; // Adjust the factor as needed
        }
        return percentReturn;
    }