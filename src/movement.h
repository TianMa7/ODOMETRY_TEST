#pragma once
// Make sure all required headers are included.

#include "vex.h"
#include "iq_cpp.h"
#include "vex_thread.h"

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
        printf("Robot initialized.\n");

        printf("I HATE VEX\n");
        wait(1000, msec);
        // create location pointer
        //  void *locationptr = &locationUpdate();
        //  startLocationThread();
    }
    // Add public methods for robot movement and odometry here
    float getLocationX()
    {
        return location[0];
    }

    float getLocationY()
    {
        return location[1];
    }

    float getHeading()
    {
        return BrainInertial.heading(degrees);
    }

    float getRotation()
    {
        return BrainInertial.rotation(degrees);
    }

    void driveTest()
    {
        // driveStraight(100, 50);
        // driveArc(50, 90, 30);
        // driveStraight(100, 50);
        // driveArc(50, 90, 30);
        // driveStraight(100, 50);
        // driveArc(50, 90, 30);
        // driveStraight(100, 50);
        // driveArc(50, 90, 30);
        driveStraight(100, 100);

        // Print final location on brain screen
    }

    // void startLocationThread()
    // {
    //     locationThread = thread(&Robot::locationUpdate);
    // }

private:
    motor &LeftMotor;
    motor &RightMotor;
    brain &Brain;
    inertial &BrainInertial;

    thread locationThread;
    float location[2] = {0, 0}; // x, y

    // CGPT: Add thread and mutex for location updates
    //  std::mutex locationMutex;                // protect location[]
    //  std::atomic<bool> stopLocationThread{false}; // request thread stop

    const double wheelCircumference = 20.0; // cm

    void configureAllSensors()
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

    void locationUpdate()
    {
        double waitTime = 10; // ms
        double lastLeft = LeftMotor.position(turns);
        double lastRight = RightMotor.position(turns);
        double lastCenter = (lastLeft + lastRight) / 2.0;

        wait(waitTime, msec); // Initial wait before starting updates

        double currentLeft = LeftMotor.position(turns);
        double currentRight = RightMotor.position(turns);
        double currentCenter = (currentLeft + currentRight) / 2.0;
        double currentHeading = BrainInertial.heading(degrees) * (M_PI / 180.0);
        double deltaCenter = (currentCenter - lastCenter) * wheelCircumference;

        // if (deltaCenter > 1 || deltaCenter < -1)
        // {
        //     printf("Warning: Large deltaCenter detected");
        // }
        // printf("PIZAA: %.2f cm\n", deltaCenter);

        location[0] += deltaCenter * cos(currentHeading); // x
        location[1] += deltaCenter * sin(currentHeading); // y // Adjust the update rate as needed
    }

    // void locationUpdate()
    // {
    //     double waitTime = 10; // ms
    //     double lastLeft = LeftMotor.position(turns);
    //     double lastRight = RightMotor.position(turns);
    //     double lastCenter = (lastLeft + lastRight) / 2.0;

    //     wait(waitTime, msec); // Initial wait before starting updates
    //     while (true)
    //     {
    //         while (!(LeftMotor.isSpinning() || RightMotor.isSpinning()))
    //         {
    //             wait(waitTime, msec);
    //         }
    //         double currentLeft = LeftMotor.position(turns);
    //         double currentRight = RightMotor.position(turns);
    //         double currentHeading = BrainInertial.heading(degrees) * (M_PI / 180.0);
    //         double currentCenter = (currentLeft + currentRight) / 2.0;
    //         double deltaCenter = (currentCenter - lastCenter) * wheelCircumference;

    //         // Update last values for next iteration
    //         lastLeft = currentLeft;
    //         lastRight = currentRight;
    //         lastCenter = currentCenter;

    //         // tests and flags
    //         if (deltaCenter > 1 || deltaCenter < -1)
    //         {
    //             printf("Warning: Large deltaCenter detected");
    //         }

    //         location[0] += deltaCenter * cos(currentHeading); // x
    //         location[1] += deltaCenter * sin(currentHeading); // y

    //         wait(waitTime, msec); // Adjust the update rate as needed
    //     }
    // }


    void moveTo(float x, float y, float speed)
    {
        // Implement path planning and movement to (x, y)
        // This is a placeholder for actual movement logic
        printf("Moving to (%.2f, %.2f) at speed %.2f\n", x, y, speed);
    }

    void driveArc(float radius, float angle, float speed)
    {
        // Implement driving in an arc with given radius and angle
        // This is a placeholder for actual arc driving logic
        printf("Driving arc with radius %.2f, angle %.2f at speed %.2f\n", radius, angle, speed);
    }

    float trackingCenter()
    {
        float leftDistance = LeftMotor.position(turns);
        float rightDistance = RightMotor.position(turns);
        float centerPosition = (leftDistance + rightDistance) / 2.0;
        return centerPosition;
    }

    void driveStraight(float distance, float maxSpeed)
    {
        // Implement driving straight for a certain distance
        // This is a placeholder for actual straight driving logic
        //   float wheelCircumference = 20.0; // cm
        float initialPosition = LeftMotor.position(turns) * wheelCircumference;

        float initialRotation = BrainInertial.rotation(degrees);
        float distanceRemaining = distance;
        float targetPosition = initialPosition + distance;
        LeftMotor.spin(forward);
        RightMotor.spin(forward);
        while ((trackingCenter() - initialPosition) * wheelCircumference < distance)
        {
            distanceRemaining = targetPosition - (trackingCenter() * wheelCircumference);
            LeftMotor.setVelocity(pLeft(distanceRemaining, maxSpeed, initialRotation), percent);
            RightMotor.setVelocity(pRight(distanceRemaining, maxSpeed, initialRotation), percent);
            locationUpdate();
        }
        LeftMotor.stop();
        RightMotor.stop();
        printf("Driving straight for distance %.2f at max speed %.2f\n", distance, maxSpeed);
    }

    float pLeft(float distanceRemaining, float maxSpeed = 100, float initialRotation = NULL)
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

    float pRight(float distanceRemaining, float maxSpeed = 100, float initialRotation = NULL)
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
};
