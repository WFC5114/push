#include <math.h>
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "lemlib/logger/logger.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/rtos.hpp"
#include <random>
#include "map.h"
#include "particle_filter.h"

ParticleFilter pf; // Particle filter
std::default_random_engine gen;
double sigma_pos[3] = {1, 1, 0.01}; // GPS measurement uncertainty [x [inches], y [inches], theta [rad]]
double sigma_landmark[2] = {1, 1}; // Landmark measurement uncertainty [x [inches], y [inches]]
extern Map map;

lemlib::OdomSensors::OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                                 TrackingWheel* horizontal2, pros::Imu* imu,
                                 std::unordered_map<std::string, std::shared_ptr<pros::Distance>>* distances)
    : vertical1(vertical1),
      vertical2(vertical2),
      horizontal1(horizontal1),
      horizontal2(horizontal2),
      imu(imu),
      distances(distances) {}

lemlib::Drivetrain::Drivetrain(pros::MotorGroup* leftMotors, pros::MotorGroup* rightMotors, float trackWidth,
                               float wheelDiameter, float rpm, float horizontalDrift)
    : leftMotors(leftMotors),
      rightMotors(rightMotors),
      trackWidth(trackWidth),
      wheelDiameter(wheelDiameter),
      rpm(rpm),
      horizontalDrift(horizontalDrift) {}

lemlib::Chassis::Chassis(Drivetrain drivetrain, ControllerSettings linearSettings, ControllerSettings angularSettings,
                         OdomSensors sensors, DriveCurve* throttleCurve, DriveCurve* steerCurve)
    : drivetrain(drivetrain),
      lateralSettings(linearSettings),
      angularSettings(angularSettings),
      sensors(sensors),
      throttleCurve(throttleCurve),
      steerCurve(steerCurve),
      lateralPID(linearSettings.kP, linearSettings.kI, linearSettings.kD, linearSettings.windupRange, true),
      angularPID(angularSettings.kP, angularSettings.kI, angularSettings.kD, angularSettings.windupRange, true),
      lateralLargeExit(lateralSettings.largeError, lateralSettings.largeErrorTimeout),
      lateralSmallExit(lateralSettings.smallError, lateralSettings.smallErrorTimeout),
      angularLargeExit(angularSettings.largeError, angularSettings.largeErrorTimeout),
      angularSmallExit(angularSettings.smallError, angularSettings.smallErrorTimeout) {}

/**
 * @brief calibrate the IMU given a sensors struct
 *
 * @param sensors reference to the sensors struct
 */
void calibrateIMU(lemlib::OdomSensors& sensors) {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        sensors.imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating());
        // exit if imu has been calibrated
        if (!isnanf(sensors.imu->get_heading()) && !isinf(sensors.imu->get_heading())) {
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        lemlib::infoSink()->warn("IMU failed to calibrate! Attempt #{}", attempt);
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) {
        sensors.imu = nullptr;
        lemlib::infoSink()->error("IMU calibration failed, defaulting to tracking wheels / motor encoders");
    }
}

void lemlib::Chassis::calibrate(bool calibrateImu) {
    // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    if (sensors.vertical1 == nullptr)
        sensors.vertical1 = new lemlib::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                      -(drivetrain.trackWidth / 2), drivetrain.rpm);
    if (sensors.vertical2 == nullptr)
        sensors.vertical2 = new lemlib::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                      drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();
    setSensors(sensors, drivetrain);
    init();
    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void lemlib::Chassis::setPose(float x, float y, float theta, bool radians) {
    lemlib::setPose(lemlib::Pose(x, y, theta), radians);
    std::normal_distribution<double> N_x_init(x, sigma_pos[0]);
    std::normal_distribution<double> N_y_init(y, sigma_pos[1]);
    std::normal_distribution<double> N_theta_init(theta, sigma_pos[2]);

    // Initialize particle filter if this is the first time step.
    pf.init(N_x_init(gen), N_y_init(gen), N_theta_init(gen), sigma_pos);
}

void lemlib::Chassis::correctAt0(std::set<std::string> sensors) { lemlib::correctAt0(sensors); }

void lemlib::Chassis::correctAt90(std::set<std::string> sensors) { lemlib::correctAt90(sensors); }

void lemlib::Chassis::correctAt180(std::set<std::string> sensors) { lemlib::correctAt180(sensors); }

void lemlib::Chassis::correctAt270(std::set<std::string> sensors) { lemlib::correctAt270(sensors); }

void lemlib::Chassis::correctByDistanceSensors() { lemlib::correctByDistanceSensors(); }

void lemlib::Chassis::setPose(Pose pose, bool radians) { lemlib::setPose(pose, radians); }

lemlib::Pose lemlib::Chassis::getPose(bool radians, bool standardPos) {
    Pose pose = lemlib::getPose(true);
    if (standardPos) pose.theta = M_PI_2 - pose.theta;
    if (!radians) pose.theta = radToDeg(pose.theta);
    return pose;
}

void lemlib::Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void lemlib::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void lemlib::Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void lemlib::Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void lemlib::Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

void lemlib::Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool lemlib::Chassis::isInMotion() const { return this->motionRunning; }

void lemlib::Chassis::resetLocalPosition() {
    float theta = this->getPose().theta;
    lemlib::setPose(lemlib::Pose(0, 0, theta), false);
}

void lemlib::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.leftMotors->set_brake_mode_all(mode);
    drivetrain.rightMotors->set_brake_mode_all(mode);
}
double lemlib::Chassis::getForwardVelocity() {
    double now = pros::millis() / 1000.0;
    double dt = now - prev_time;
    if (dt <= 0) dt = 0.02;

    double delta_distance = 0;

    if (sensors.vertical1 && sensors.vertical2) {
        double left_pos = sensors.vertical1->getDistanceTraveled();
        double right_pos = sensors.vertical2->getDistanceTraveled();
        delta_distance = ( (left_pos - prev_left) + (right_pos - prev_right) ) / 2.0;
        prev_left = left_pos;
        prev_right = right_pos;
    } else if (sensors.vertical1) {
        double left_pos = sensors.vertical1->getDistanceTraveled();
        delta_distance = left_pos - prev_left;
        prev_left = left_pos;
    } else if (sensors.vertical2) {
        double right_pos = sensors.vertical2->getDistanceTraveled();
        delta_distance = right_pos - prev_right;
        prev_right = right_pos;
    } else {
        delta_distance = 0; // no wheel info available
    }

    prev_time = now;
    return delta_distance / dt;
}

double lemlib::Chassis::getYawRate() {
    double now = pros::millis() / 1000.0;
    double dt = now - prev_time;
    if (dt <= 0) dt = 0.02;

    double delta_yaw = 0;

    if (sensors.imu) {
        double yaw = sensors.imu->get_rotation();
        delta_yaw = (yaw - prev_yaw) * M_PI / 180.0; // convert to radians
        prev_yaw = yaw;
    } else if (sensors.vertical1 && sensors.vertical2) {
        double left_pos = sensors.vertical1->getDistanceTraveled();
        double right_pos = sensors.vertical2->getDistanceTraveled();
        delta_yaw = ((right_pos - prev_right) - (left_pos - prev_left)) / drivetrain.trackWidth;
        prev_left = left_pos;
        prev_right = right_pos;
    } else {
        delta_yaw = 0; // no yaw info available
    }

    prev_time = now;
    return delta_yaw / dt;
}
