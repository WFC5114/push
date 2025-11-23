#pragma once

// Auton Selection
constexpr bool autonSelector = false;

// Arm
constexpr float MAX_DEGREES = 23000.0f; // 150 degrees
constexpr float WAIT_DEGREES = 2500.0f; // 35 degrees
constexpr float DOWN_DEGREES = 1500.0f; 
constexpr float VERTICAL_DEGREES = 11000;
constexpr float DESCORE_DEGREES = 14500; // 150 degrees
constexpr float SCORE_DEGREES = 16000;
constexpr float AUTO_DEGREES = 20500;


// Holder
constexpr int AUTO_HOLD_TIMEOUT = 1000; // 1 sec
static bool activeAutoClamp = true;
inline bool withAutoClamp = true;

// Odom
constexpr int MAX_DIST_INCHES = 48;
constexpr int MAX_DIST_MM = 1000;

constexpr double BACK_OFFSET_INCHES = 5;
constexpr double LEFT_OFFSET_INCHES = 5.75;
constexpr double RIGHT_OFFSET_INCHES = 5.75;
constexpr double FRONT_OFFSET_INCHES = 5;

constexpr double FEILD_SIZE = 71;
 
// Conveyor
inline bool withAntiJam = false;
constexpr double RED_HIGH_THRESHOLD = 340;
constexpr double BLUE_HIGH_THRESHOLD = 270;

constexpr double RED_THRESHOLD = 15; // detect red (HIGH_THRESHOLD, 360) and (0, RED_THRESHOLD)
constexpr double BLUE_THRESHOLD = 160; // detect blue (BLUE_THRESHOLD, HIGH_THRESHOLD)
constexpr int PROXIMITY_THRESHOLD = 100;
constexpr int STUCK_CURRENT = 2000;
constexpr int STUCK_RPM = 20;
constexpr double STUCK_TORQUE = 0.7;

constexpr double SPEEDRATIO = 1.0;
constexpr double DIST_THRESHOLD = 40;
constexpr int check_interval = 10;

//constexpr bool AUTO_STARTED = true;

//Skills
constexpr bool RUN_SKILLS = false;


