#pragma once
// Make sure all required headers are included.
#include "vex.h"

using namespace vex;

class Robot
{

public:
    Robot(
        motor &Left,
        motor &Right,
        brain &Brain,
        inertial &IMU)
        : MotorLeft(Left),
          MotorRight(Right),
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

    void moveTo(float x, float y, float finalAngle, float speed);

    void locationUpdate();

private:
    motor &MotorLeft;
    motor &MotorRight;
    brain &Brain;
    inertial &BrainInertial;

    float location[2] = {0, 0};             // x, y
    const double wheelCircumference = 20.0; // cm
    const double wheelBase = 14.5;             // cm

    void configureAllSensors();

    float arcRatio(int radius)
    {
        return (1 - (wheelBase / (2 * radius))) / (1 + (wheelBase / (2 * radius)));
    }

    float findTangent(float centerX, float centerY, float radius)
    {
        return (tan((centerX - location[0]) / (centerY - location[1])) - tan((radius) / (hypot(centerY - location[1], centerX - location[0])))) * 180 / M_PI;
    }

    // MATH EXPLAINED BY CHATGPT I COULDNT THINK CLEAR ENOUGH
    float normalizeAngle(float robotAngle, float targetAngle)
    {
        float angle = targetAngle - robotAngle;
        if (angle >= 180)
        {
            angle -= 360;
        }
        if (angle < -180)
        {
            angle += 360;
        }
        return angle;
    }
};
