#pragma once
// Make sure all required headers are included.
#include "vex.h"

using namespace vex;

class Robot
{
    // Movement class implementation details
    // Outside should be able to access:
    // getLocation(), moveTo(x, y), driveArc(radius, angle, speed)
public:
    Robot(
        motor &Left,
        motor &Right,
        brain &Brain,
        inertial &IMU)
        : LeftMotor(Left),
          RightMotor(Right),
          Brain(Brain),
          BrainInertial(IMU)
    {
        configureAllSensors();
        wait(1000, msec);
    }
    // Add public methods for robot movement and odometry here
    float getX() const
    {
        return location[0];
    }

    float getY() const
    {
        return location[1];
    }

    float getHeading() const
    {
        return BrainInertial.heading(degrees);
    }

    float getRotation() const
    {
        return BrainInertial.rotation(degrees);
    }

    void moveTo(float x, float y, float speed);

    void driveTest();

private:
    motor &LeftMotor;
    motor &RightMotor;
    brain &Brain;
    inertial &BrainInertial;

    float location[2] = {0, 0};             // x, y
    const double wheelCircumference = 20.0; // cm
    const double wheelBase = 0; //cm

    void configureAllSensors();
    void locationUpdate();
    void driveArc(float radius, float angle, float speed);
    void driveStraight(float distance, float speed);
    float pLeft(float distanceRemaining, float maxSpeed, float initialRotation);
    float pRight(float distanceRemaining, float maxSpeed, float initialRotation);
    // UNSURE IF NEEDED
    float trackingCenter();

    float arcRatio(int radius)
    {
        return (1 - (wheelBase/ (2 * radius))) / (1 + (wheelBase/ (2 * radius)));
    }
};
