#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

/**
 * @brief Move the robot forward a specified distance
 * 
 * @param dist Distance to move (same unit as wheel diameter, e.g., inches)
 * @param timeout Maximum time allowed (ms)
 * @param params MoveToPointParams containing minSpeed, maxSpeed, forwards, earlyExitRange
 * @param async Whether to run asynchronously
 */
void lemlib::Chassis::moveForward(float dist, int timeout, MoveToPointParams params, bool async) {
    // Ensure earlyExitRange is positive
    params.earlyExitRange = fabs(params.earlyExitRange);

    this->requestMotionStart();
    if (!this->motionRunning) return;

    // If async, run in a new task
    if (async) {
        pros::Task task([&]() {
            moveForward(dist, timeout, params, false);
        });
        this->endMotion();
        pros::delay(10);
        return;
    }

    // Reset PID controllers and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();

    // Initial pose and distance
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    float prevLateralOut = 0;
    bool close = false;
    float initTheta = getPose(true, true).theta;

    if (dist < 0) {
        dist = -dist;
        params.forwards = false;
    }

    // Function to get average distance traveled by left/right motors
    auto getDistanceTraveled = [this]() -> float {
        float leftDeg = drivetrain.leftMotors->get_position();
        float rightDeg = drivetrain.rightMotors->get_position();
        float avgDeg = (leftDeg + rightDeg) / 2.0f;
        float wheelCircumference = M_PI * drivetrain.wheelDiameter; // same unit as dist
        return avgDeg / 360.0f * wheelCircumference;
    };

    float initDistanceTraveled = getDistanceTraveled();

    // Heading PID to maintain straight line
    lemlib::PID headingPID(2.0, 0.0, 12.0); // tune for your chassis

    // Main loop
    while (!timer.isDone() && ((!lateralSmallExit.getExit() && !lateralLargeExit.getExit()) || !close) &&
           this->motionRunning) {

        // Update distance traveled
        distTraveled = getDistanceTraveled() - initDistanceTraveled;

        // Distance remaining to target
        float distTarget = dist - distTraveled;

        // Check if close to target for slowing down
        if (distTarget < 7.5f && !close) {  // 7.5 units threshold
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60.0f);  // slow down max speed
        }

        // Compute lateral error
        float lateralError = distTarget;

        // Update exit conditions
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);

        // Lateral PID output
        float lateralOut = lateralPID.update(lateralError);

        // Clamp output to maxSpeed
        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

        // Apply slew rate limiting for smooth acceleration/deceleration
        lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

        // Prevent moving in wrong direction
        if (params.forwards && !close) lateralOut = std::fmax(lateralOut, 0);
        if (!params.forwards && !close) lateralOut = std::fmin(lateralOut, 0);

        // Enforce minimum speed
        if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0)
            lateralOut = fabs(params.minSpeed);
        if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
            lateralOut = -fabs(params.minSpeed);

        prevLateralOut = lateralOut;

        // Heading correction
        float currentTheta = getPose(true, true).theta;
        float headingError = lemlib::wrapAngle(initTheta - currentTheta);
        float headingOut = headingPID.update(headingError);
        headingOut = std::clamp(headingOut, -20.0f, 20.0f);

        if (close) headingOut = 0;  // disable heading correction near target

        // Apply heading correction
        float leftPower = lateralOut + headingOut;
        float rightPower = lateralOut - headingOut;

        // Normalize ratio to stay within maxSpeed
        float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
        if (ratio > 1) {
            leftPower /= ratio;
            rightPower /= ratio;
        }

        // Drive motors
        drivetrain.leftMotors->move(leftPower);
        drivetrain.rightMotors->move(rightPower);

        pros::delay(10);
    }

    // Stop drivetrain (COAST mode remains)
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);

    distTraveled = -1;
    this->endMotion();
}
