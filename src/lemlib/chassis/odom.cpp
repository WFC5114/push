// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <math.h>
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "lemlib/util.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "constants.hpp"
#include <random>
#include "map.h"
#include "particle_filter.h"

// tracking thread
pros::Task* trackingTask = nullptr;

// global variables
lemlib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr,
                                nullptr); // the sensors to be used for odometry
lemlib::Drivetrain drive(nullptr, nullptr, 0, 0, 0, 0); // the drivetrain to be used for odometry
lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
lemlib::Pose odomSpeed(0, 0, 0); // the speed of the robot
lemlib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot
lemlib::Timer odomTimer {10000}; // the timer for correction of odm
extern ParticleFilter pf; // Particle filter
extern std::default_random_engine gen;
extern double sigma_pos[3];
extern double sigma_landmark[2];
// Static constant instance of the map with updated boundaries
const Map map = [] {
    Map m;

    // Define the field boundaries for x and y (from -72 to 72)
    m.min_x = -72;
    m.max_x = 72;
    m.min_y = -72;
    m.max_y = 72;

    // Define the obstacles (landmarks)
    m.landmark_list = {
        {1, 24.0, 0.0}, // Obstacle at (24, 0)
        {2, 0.0, 24.0}, // Obstacle at (0, 24)
        {3, -24.0, 0.0}, // Obstacle at (-24, 0)
        {4, 0.0, -24.0} // Obstacle at (0, -24)
    };

    return m;
}();

float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;
float distanceTraveledAccumulator = 0;

void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

lemlib::Pose lemlib::getPose(bool radians) {
    if (radians) return odomPose;
    else return lemlib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

void lemlib::setPose(lemlib::Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = lemlib::Pose(pose.x, pose.y, degToRad(pose.theta));
}

lemlib::Pose lemlib::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    else return lemlib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

lemlib::Pose lemlib::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    else return lemlib::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

lemlib::Pose lemlib::estimatePose(float time, bool radians) {
    // get current position and speed
    Pose curPose = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    // calculate the change in local position
    Pose deltaLocalPose = localSpeed * time;

    // calculate the future pose
    float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalPose.y * sin(avgHeading);
    futurePose.y += deltaLocalPose.y * cos(avgHeading);
    futurePose.x += deltaLocalPose.x * -cos(avgHeading);
    futurePose.y += deltaLocalPose.x * sin(avgHeading);
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

void lemlib::correctByDistanceSensors() {
    // Check if the distance sensor data is available.
    if (odomSensors.distances == nullptr) { return; }

    // For convenience, alias the distances map.
    auto distances = odomSensors.distances;

    // Copy the current odometry (turning center) and heading.
    double x = odomPose.x;
    double y = odomPose.y;
    double theta = odomPose.theta;

    // --- Correction using the back sensor (for the back wall at y = -FEILD_SIZE) ---
    // The back sensor is mounted at (0, -BACK_OFFSET_INCHES) in the robot frame.
    if (distances->find("back") != distances->end()) {
        double back_mm = distances->at("back")->get_distance();
        if (back_mm < MAX_DIST_MM) {
            double backInches = back_mm / 25.4;
            // backInches = y_B - (-FEILD_SIZE) where y_B = y - BACK_OFFSET_INCHES*cos(theta)
            // So, y - BACK_OFFSET_INCHES*cos(theta) = -FEILD_SIZE + backInches
            // Rearranging gives:
            y = -FEILD_SIZE + backInches + BACK_OFFSET_INCHES * cos(theta);
        }
    }

    // --- Correction using the front sensor (for the front wall at y = FEILD_SIZE) ---
    // The front sensor is mounted at (0, FRONT_OFFSET_INCHES) in the robot frame.
    if (distances->find("front") != distances->end()) {
        double front_mm = distances->at("front")->get_distance();
        if (front_mm < MAX_DIST_MM) {
            double frontInches = front_mm / 25.4;
            // frontInches = FEILD_SIZE - y_F where y_F = y + FRONT_OFFSET_INCHES*cos(theta)
            // So, y + FRONT_OFFSET_INCHES*cos(theta) = FEILD_SIZE - frontInches
            // Rearranging gives:
            y = FEILD_SIZE - frontInches - FRONT_OFFSET_INCHES * cos(theta);
        }
    }

    // --- Correction using the left sensor (for the left wall at x = -FEILD_SIZE) ---
    // The left sensor is mounted at (-LEFT_OFFSET_INCHES, 0) in the robot frame.
    if (distances->find("left") != distances->end()) {
        double left_mm = distances->at("left")->get_distance();
        if (left_mm < MAX_DIST_MM) {
            double leftInches = left_mm / 25.4;
            // leftInches = x_L - (-FEILD_SIZE) where x_L = x - LEFT_OFFSET_INCHES*cos(theta)
            // So, x - LEFT_OFFSET_INCHES*cos(theta) = -FEILD_SIZE + leftInches
            // Rearranging gives:
            x = -FEILD_SIZE + leftInches + LEFT_OFFSET_INCHES * cos(theta);
        }
    }

    // --- Correction using the right sensor (for the right wall at x = FEILD_SIZE) ---
    // The right sensor is mounted at (RIGHT_OFFSET_INCHES, 0) in the robot frame.
    if (distances->find("right") != distances->end()) {
        double right_mm = distances->at("right")->get_distance();
        if (right_mm < MAX_DIST_MM) {
            double rightInches = right_mm / 25.4;
            // rightInches = FEILD_SIZE - x_R where x_R = x + RIGHT_OFFSET_INCHES*cos(theta)
            // So, x + RIGHT_OFFSET_INCHES*cos(theta) = FEILD_SIZE - rightInches
            // Rearranging gives:
            x = FEILD_SIZE - rightInches - RIGHT_OFFSET_INCHES * cos(theta);
        }
    }

    // Update the pose with the corrected x, y, and the current theta.
    setPose(lemlib::Pose(x, y, theta), true);
}

lemlib::Pose lemlib::bestPoe() {
    Particle best_particle;
    double highest_weight = 0.0;
    for (int i = 0; i < pf.num_particles; ++i) {
        if (pf.particles[i].weight > highest_weight) {
            highest_weight = pf.particles[i].weight;
            best_particle = pf.particles[i];
        }
    }
    return lemlib::Pose(best_particle.x, best_particle.y, best_particle.theta);
}

lemlib::Pose lemlib::estimatePose() {
    double sum_x = 0.0, sum_y = 0.0, sum_theta = 0.0, sum_weight = 0.0;

    for (const auto& p : pf.particles) {
        sum_x += p.x * p.weight;
        sum_y += p.y * p.weight;
        sum_theta += p.theta * p.weight;
        sum_weight += p.weight;
    }

    // Check to avoid divide-by-zero
    if (fabs(sum_weight) < 1e-9) {
        // Handle the error: here we choose to return a default pose (or you could return odomPose or any fallback)
        return odomPose;
    }
    return lemlib::Pose(sum_x / sum_weight, sum_y / sum_weight, sum_theta / sum_weight);
}

void lemlib::correctAt0(std::set<std::string> sensors) {
    if (odomSensors.distances) {
        auto distances = odomSensors.distances;
        auto x = odomPose.x;
        auto y = odomPose.y;

        if (distances->find("front") != distances->end() and sensors.count("front") > 0) {
            auto front = distances->at("front")->get_distance();
            if (distances->at("front")->get_confidence() > 40) y = 71 - front / 25.4 - FRONT_OFFSET_INCHES;
        }
        if (distances->find("back") != distances->end() and sensors.count("back") > 0) {
            auto back = distances->at("back")->get_distance();
            if (distances->at("back")->get_confidence() > 40) y = -71 + back / 25.4 + BACK_OFFSET_INCHES;
        }
        if (distances->find("left") != distances->end() and sensors.count("left") > 0) {
            auto left = distances->at("left")->get_distance();
            if (distances->at("left")->get_confidence() > 40) x = -71 + left / 25.4 + LEFT_OFFSET_INCHES;
        }
        if (distances->find("right") != distances->end() and sensors.count("right") > 0) {
            auto right = distances->at("right")->get_distance();
            if (distances->at("right")->get_confidence() > 40) x = 71 - right / 25.4 - RIGHT_OFFSET_INCHES;
        }

        setPose(lemlib::Pose(x, y, odomPose.theta), true);
    }
}

void lemlib::correctAt90(std::set<std::string> sensors) {
    if (odomSensors.distances) {
        auto distances = odomSensors.distances;
        auto x = odomPose.x;
        auto y = odomPose.y;

        if (distances->find("front") != distances->end() and sensors.count("front") > 0) {
            auto front = distances->at("front")->get_distance();
            if (distances->at("front")->get_confidence() > 40) x = 71 - front / 25.4 - FRONT_OFFSET_INCHES;
        }
        if (distances->find("back") != distances->end() and sensors.count("back") > 0) {
            auto back = distances->at("back")->get_distance();
            if (distances->at("back")->get_confidence() > 40) x = -71 + back / 25.4 + BACK_OFFSET_INCHES;
        }
        if (distances->find("left") != distances->end() and sensors.count("left") > 0) {
            auto left = distances->at("left")->get_distance();
            if (distances->at("left")->get_confidence() > 40) y = 71 - left / 25.4 - LEFT_OFFSET_INCHES;
        }
        if (distances->find("right") != distances->end() and sensors.count("right") > 0) {
            auto right = distances->at("right")->get_distance();
            if (distances->at("right")->get_confidence() > 40) y = -71 + right / 25.4 + RIGHT_OFFSET_INCHES;
        }

        setPose(lemlib::Pose(x, y, odomPose.theta), true);
    }
}

void lemlib::correctAt180(std::set<std::string> sensors) {
    if (odomSensors.distances) {
        auto distances = odomSensors.distances;
        auto x = odomPose.x;
        auto y = odomPose.y;

        if (distances->find("front") != distances->end() and sensors.count("front") > 0) {
            auto front = distances->at("front")->get_distance();
            if (distances->at("front")->get_confidence() > 40) y = -71 + front / 25.4 + FRONT_OFFSET_INCHES;
        }
        if (distances->find("back") != distances->end() and sensors.count("back") > 0) {
            auto back = distances->at("back")->get_distance();
            if (distances->at("back")->get_confidence() > 40) y = 71 - back / 25.4 - BACK_OFFSET_INCHES;
        }
        if (distances->find("left") != distances->end() and sensors.count("left") > 0) {
            auto left = distances->at("left")->get_distance();
            if (distances->at("left")->get_confidence() > 40) x = 71 - left / 25.4 - LEFT_OFFSET_INCHES;
        }
        if (distances->find("right") != distances->end() and sensors.count("right") > 0) {
            auto right = distances->at("right")->get_distance();
            if (distances->at("right")->get_confidence() > 40) x = -71 + right / 25.4 + RIGHT_OFFSET_INCHES;
        }

        setPose(lemlib::Pose(x, y, odomPose.theta), true);
    }
}

void lemlib::correctAt270(std::set<std::string> sensors) {
    if (odomSensors.distances) {
        auto distances = odomSensors.distances;
        auto x = odomPose.x;
        auto y = odomPose.y;

        if (distances->find("front") != distances->end() and sensors.count("front") > 0) {
            auto front = distances->at("front")->get_distance();
            if (distances->at("front")->get_confidence() > 40) x = -71 + front / 25.4 + FRONT_OFFSET_INCHES;
        }
        if (distances->find("back") != distances->end() and sensors.count("back") > 0) {
            auto back = distances->at("back")->get_distance();
            if (distances->at("back")->get_confidence() > 40) x = 71 - back / 25.4 - BACK_OFFSET_INCHES;
        }
        if (distances->find("left") != distances->end() and sensors.count("left") > 0) {
            auto left = distances->at("left")->get_distance();
            if (distances->at("left")->get_confidence() > 40) y = -71 + left / 25.4 + LEFT_OFFSET_INCHES;
        }
        if (distances->find("right") != distances->end() and sensors.count("right") > 0) {
            auto right = distances->at("right")->get_distance();
            if (distances->at("right")->get_confidence() > 40) y = 71 - right / 25.4 - RIGHT_OFFSET_INCHES;
        }

        setPose(lemlib::Pose(x, y, odomPose.theta), true);
    }
}

float lemlib::getDistanceTraveled() {
    // Get the distance traveled by the robot since the last update.   
    return  distanceTraveledAccumulator;
}


void lemlib::update() {
    // 1) Get the current sensor values.
    float vertical1Raw = (odomSensors.vertical1) ? odomSensors.vertical1->getDistanceTraveled() : 0;
    float vertical2Raw = (odomSensors.vertical2) ? odomSensors.vertical2->getDistanceTraveled() : 0;
    float horizontal1Raw = (odomSensors.horizontal1) ? odomSensors.horizontal1->getDistanceTraveled() : 0;
    float horizontal2Raw = (odomSensors.horizontal2) ? odomSensors.horizontal2->getDistanceTraveled() : 0;
    float imuRaw = (odomSensors.imu) ? degToRad(odomSensors.imu->get_rotation()) : 0;

    // 2) Calculate the change in sensor values.
    // (These deltas represent the incremental change since the last update.)
    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw - prevImu;

    // 3) Update previous sensor readings for next update cycle.
    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu = imuRaw;

    // 4) Compute heading using available sensors.
    // Priority:
    //   a) If both horizontal tracking wheels are available, use their delta.
    //   b) Else if both vertical wheels are available and non-driven, use them.
    //   c) Else if IMU is available, use it.
    //   d) Else fallback to using vertical wheels.
    float heading = odomPose.theta;
    // calculate the heading using the horizontal tracking wheels
    if (odomSensors.horizontal1 != nullptr && odomSensors.horizontal2 != nullptr)
        heading -= (deltaHorizontal1 - deltaHorizontal2) /
                   (odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset());
    // else, if both vertical tracking wheels aren't substituted by the drivetrain, use the vertical tracking wheels
    else if (!odomSensors.vertical1->getType() && !odomSensors.vertical2->getType())
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    // else, if the inertial sensor exists, use it
    else if (odomSensors.imu != nullptr) heading += deltaImu;
    // else, use the the substituted tracking wheels
    else
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    // Calculate change in heading and average heading for the time step.
    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2.0;

    // choose tracking wheels to use
    // Prioritize non-powered tracking wheels
    lemlib::TrackingWheel* verticalWheel = nullptr;
    lemlib::TrackingWheel* horizontalWheel = nullptr;
    if (!odomSensors.vertical1->getType()) verticalWheel = odomSensors.vertical1;
    else if (!odomSensors.vertical2->getType()) verticalWheel = odomSensors.vertical2;
    else verticalWheel = odomSensors.vertical1;
    if (odomSensors.horizontal1 != nullptr) horizontalWheel = odomSensors.horizontal1;
    else if (odomSensors.horizontal2 != nullptr) horizontalWheel = odomSensors.horizontal2;
    float rawVertical = 0;
    float rawHorizontal = 0;
    if (verticalWheel != nullptr) rawVertical = verticalWheel->getDistanceTraveled();
    if (horizontalWheel != nullptr) rawHorizontal = horizontalWheel->getDistanceTraveled();
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr) horizontalOffset = horizontalWheel->getOffset();

    // 5) Compute local motion (displacement) using one set of tracking wheels.
    // (Here, we use vertical1 and horizontal1 as the primary sensors.)
    float deltaX = 0;
    float deltaY = 0;
    if (verticalWheel != nullptr) deltaY = rawVertical - prevVertical;
    if (horizontalWheel != nullptr) deltaX = rawHorizontal - prevHorizontal;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    // 6) Calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        // If heading change is negligible, assume straight-line motion.
        localX = deltaX;
        localY = deltaY;
    } else {
        // Use the approximation that accounts for circular arc motion.
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // save previous pose
    lemlib::Pose prevPose = odomPose;

    // Calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.y += localY * cos(avgHeading);
    odomPose.x += localX * -cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);
    odomPose.theta = heading;
    // calculate the distance traveled
    distanceTraveledAccumulator += sqrt(localX * localX + localY * localY);
    // 7) Calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);

    // 8) Calculate local speed
    odomLocalSpeed.x = ema(localX / 0.01, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / 0.01, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / 0.01, odomLocalSpeed.theta, 0.95);

    // 9) Particle Filter: Prediction step.
    // Noise for the prediction step. Adjust these as needed for your units.
    double localSpeed = sqrt(odomLocalSpeed.x * odomLocalSpeed.x + odomLocalSpeed.y * odomLocalSpeed.y);
    pf.prediction(0.01, sigma_pos, localSpeed, odomLocalSpeed.theta);

    // 10) Particle Filter: Measurement update.
    // Build landmark observations from distance sensors.
    std::vector<LandmarkObs> observations;
    std::normal_distribution<double> N_obs_x(localX, sigma_landmark[0]);
    std::normal_distribution<double> N_obs_y(localY, sigma_landmark[1]);

    if (odomSensors.distances) {
        auto distances = odomSensors.distances;

        // Front sensor measurement (along +y).
        if (distances->find("front") != distances->end()) {
            double front_mm = distances->at("front")->get_distance();
            LandmarkObs obs {static_cast<int>(observations.size() + 1), N_obs_x(gen), N_obs_y(gen)};
            // Convert sensor reading from mm to inches and apply the front mounting offset.
            obs.x += 0;
            obs.y += front_mm / 25.4 + FRONT_OFFSET_INCHES;
            observations.push_back(obs);
        }

        // Back sensor measurement (along -y).
        if (distances->find("back") != distances->end()) {
            double back_mm = distances->at("back")->get_distance();
            LandmarkObs obs {static_cast<int>(observations.size() + 1), N_obs_x(gen), N_obs_y(gen)};
            obs.x += 0;
            // Negative direction for back sensor plus its mounting offset.
            obs.y += -back_mm / 25.4 - BACK_OFFSET_INCHES;
            observations.push_back(obs);
        }

        // Right sensor measurement (along +x).
        if (distances->find("right") != distances->end()) {
            double right_mm = distances->at("right")->get_distance();
            LandmarkObs obs {static_cast<int>(observations.size() + 1), N_obs_x(gen), N_obs_y(gen)};
            // Convert reading and apply the right sensor offset.
            obs.x += right_mm / 25.4 + RIGHT_OFFSET_INCHES;
            obs.y += 0;
            observations.push_back(obs);
        }

        // Left sensor measurement (along -x).
        if (distances->find("left") != distances->end()) {
            double left_mm = distances->at("left")->get_distance();
            LandmarkObs obs {static_cast<int>(observations.size() + 1), N_obs_x(gen), N_obs_y(gen)};
            obs.x += -left_mm / 25.4 - LEFT_OFFSET_INCHES;
            obs.y += 0;
            observations.push_back(obs);
        }
    }

    // Update particle filter with the new measurements.
    pf.updateWeights(MAX_DIST_INCHES, sigma_landmark, observations, map);
    pf.resample();

    // Optionally, you can update your odometry pose with a fused estimate from the particle filter.
    // For example:
    if (odomTimer.isDone()) {
        // odomPose = bestPoe();
        // correctByDistanceSensors();
        odomTimer.reset();
    }
}

void lemlib::init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}
