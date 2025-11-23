#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::turnAndMoveToPoint(float x, float y, int timeout, MoveToPointParams params, bool async) {
    if (params.forwards == false) turnToPoint(x, y, timeout, {.forwards = false}, async);
    else turnToPoint(x, y, timeout, {}, async);
    moveToPoint(x, y, timeout, params, async);
}
