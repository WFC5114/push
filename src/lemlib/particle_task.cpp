#include "particle_task.hpp"
#include <cmath>
#include <iostream>
#include "map.h"

extern Map map_landmarks; // declare the global variable

void ParticleTask::init(double x, double y, double theta) {
    double std[] = {1.0, 1.0, 0.05}; // Initial standard deviations
    pf.init(x, y, theta, std);
}

void ParticleTask::start() {
    if (task_ptr == nullptr) {
        task_ptr = new pros::Task([this]() { taskLoop(); });
    }
}

void ParticleTask::taskLoop() {
    double delta_t = 0.02; // 50 Hz
    double std_pos[] = {0.05, 0.05, 0.01};
    double sensor_range = 50.0;
    double std_landmark[] = {0.3, 0.3};

    while (true) {
        // get velocity and yaw rate from chassis
        double velocity = chassis.getForwardVelocity();
        double yaw_rate = chassis.getYawRate();

        pf.prediction(delta_t, std_pos, velocity, yaw_rate);

        // get observations from sensors
        std::vector<LandmarkObs> observations;
        // fill observations

        pf.updateWeights(sensor_range, std_landmark, observations, map);
        pf.resample();

        // obtain best particle pose for display
        auto pose = pf.getBestParticlePose(); // need to implement this method in ParticleFilter
        // pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 0, "X: %.2f Y: %.2f Theta: %.2f", 
        //                     pose.x, pose.y, pose.theta);

        pros::delay(20);
    }
}
