#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/optical.hpp"

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
inline pros::MotorGroup leftMotors({-18, -20, -19},
                            pros::MotorGearset::blue); // left motor group - ports 10, 8 (reversed), 7
inline pros::MotorGroup rightMotors({17, 16, 15},
                             pros::MotorGearset::blue); // right motor group - ports 1 (reversed), 2 , 3 (reversed)

inline lemlib::ControllerSettings armAngularController(2, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                10, // derivative gain (kD)
                                                3, // anti windup
                                                0.5, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
                                                1, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
);

inline lemlib::ControllerSettings armAngularControllerSmallAngle(2, // proportional gain (kP)
                                                0, // integral gain (kI)
                                                10, // derivative gain (kD)
                                                3, // anti windup
                                                0.5, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
                                                1, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
);

// Other Subsystems
inline std::unordered_map<std::string, std::shared_ptr<pros::Distance>> distances {
    {"left", std::make_shared<pros::Distance>(0)},
    {"right", std::make_shared<pros::Distance>(0)},
    {"front", std::make_shared<pros::Distance>(0)}};

inline pros::Optical ColorSensor(3); // color sensor - port 3
inline ArmNamespace::Arm arm(std::make_shared<pros::Motor>(0, pros::v5::MotorGears::red), // arm - motor port 5 (reversed)
                      std::make_shared<pros::Rotation>(0), // rotation sensor - port 17 (reversed)
                      armAngularController, armAngularControllerSmallAngle); // PID controller
inline SpinnerNamespace::Spinner intake(std::make_shared<pros::Motor>(13, pros::v5::MotorGears::blue)); // intake - motor port 7
inline SpinnerNamespace::Spinner hood(std::make_shared<pros::Motor>(-14, pros::v5::MotorGears::blue)); // hooks - motor port 11
inline ConveyorNamespace::Conveyor conveyor(&intake, // intake
                                     &hood, // hooks
                                     std::make_shared<pros::Optical>(0) // optical sensor - port 14
                                   // distance sensor - port 20
);
inline HolderNamespace::Holder goal_transfer(std::make_shared<pros::adi::DigitalOut>('F')); // mobile goal holder - port 'B'
inline HolderNamespace::Holder Lipper(std::make_shared<pros::adi::DigitalOut>('A')); // doinker - port 'A'
inline HolderNamespace::Holder Descore(std::make_shared<pros::adi::DigitalOut>('H')); // hanger - port 'H'
inline HolderNamespace::Holder deadwheel_lifter(std::make_shared<pros::adi::DigitalOut>('B')); // deadwheel lifter - port 'C'


// Inertial Sensor on port 21
inline pros::Imu imu(12);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, reversed
inline pros::Rotation horizontalEnc(7);
// vertical tracking wheel encoder. Rotation sensor, port 19, reversed
inline pros::Rotation verticalEnc(8);
// horizontal tracking wheel. 2" diameter, 3.625" offset, back of the robot (negative)
inline lemlib::TrackingWheel horizontal(&horizontalEnc, 2.75, -2.625);
// vertical tracking wheel. 2" diameter, 0.5" offset, left of the robot (negative)
inline lemlib::TrackingWheel vertical(&verticalEnc, 2.75, -0.695);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              3.25, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 8. If we had traction wheels, it would have been 8
);

// lateral motion controller
inline lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            0.5, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            1, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             0.5, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             1, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
inline lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu, // inertial sensor
                            &distances // distance sensors
);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


// angular motion controller
inline lemlib::ControllerSettings angularControllerSmall(1.1, // proportional gain (kP)
                                             0.05, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             3, // anti windup
                                             0.5, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             1, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// create the chassis
inline lemlib::Chassis chassisSmall(drivetrain, linearController, angularControllerSmall, sensors, &throttleCurve, &steerCurve);