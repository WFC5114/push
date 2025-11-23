#pragma once
#include "map.h"
#include "particle_filter.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/rtos.hpp"
#include <vector>


extern Map map_landmarks;

class ParticleTask {
    lemlib::Chassis& chassis ;
    Map& map;
public:
    ParticleTask(lemlib::Chassis& chassis, Map& map) : chassis(chassis), map(map) {}
    void init(double x, double y, double theta);

    void start();

private:
    ParticleFilter pf;
    pros::Task* task_ptr = nullptr;

    void taskLoop(); 
};
