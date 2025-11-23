
#include "components.hpp"
#include "auton/utility.hpp"
#include "constants.hpp"
#include "map.h"
#include "particle_task.hpp"


Map map_landmarks; 
ParticleTask particleTask(chassis, map_landmarks);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    chassis.calibrate(); // calibrate sensors
    conveyor.getOpticalSensor()->set_led_pwm(100);
    
    double initial_x = chassis.getPose().x;
    double initial_y = chassis.getPose().y;
    double initial_theta = chassis.getPose().theta;
    double std[] = {1.0, 1.0, 0.05}; 
    particleTask.init(initial_x, initial_y, initial_theta);
    particleTask.start();


    if (RUN_SKILLS) conveyor.disable_color_sensor();
    // Selector callback example, prints selected auton to the console
    if(autonSelector){
        selector.on_select([](std::optional<rd::Selector::routine_t> routine) {
            if (routine == std::nullopt) {
                std::cout << "No routine selected" << std::endl;
            } else {
                std::cout << "Selected Routine: " << routine.value().name << std::endl;
            }
        });
    }
    // thread to for brain screen and position logging
    if(autonSelector == false){
        pros::Task screenTask   ([&]() {
            while (true) {
                pros::screen::erase();
                // print robot location to the brain screen
                pros::screen::print(pros::text_format_e_t::E_TEXT_MEDIUM_CENTER, 0, "X: %f, Y: %f, Theta: %f",
                                    chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta); // x

                pros::delay(50);
            }
        });
    }
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

void autonomous() {
    // skills();
 
    if (withAntiJam) {
        pros::Task antiJam([&]() {
            while (withAntiJam) {
                if (conveyor.getIntake()->getMotor()->get_actual_velocity() > 100 && conveyor.getHood()->getMotor()->get_actual_velocity() < 50 && conveyor.getHood()->getMotor()->get_current_draw() > 1500) {
                    ConveyorNamespace::State curState = conveyor.getState();
                    conveyor.moveToState(ConveyorNamespace::State::OUTTAKE);
                    pros::delay(100);
                    conveyor.moveToState(curState);
                }
                pros::delay(20);
            }
            pros::delay(20);
        });
    }


    if (autonSelector) {
        if (RUN_SKILLS) skills(); 
        else selector.run_auton();

    }
    else  {
        blue_left();
        //blue_7_right();
       //red_left();
       //red_7_right();


    }
    return;

}

/**
 * Runs in driver control
 */
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    lemlib::Timer timer(15000);
    conveyor.disable_color_sensor();
    deadwheel_lifter.moveToState(HolderNamespace::State::HOLD);
    // controller
    // loop to continuously update motors
    while (true) {
        if (timer.isDone()) {
            controller.rumble(".-.-");
            timer.reset();
        }

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // Get button states with clearer variable names
        bool buttonR2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool buttonY = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
        bool buttonL1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool buttonL2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool buttonR1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool buttonA = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
        bool buttonUp = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
        bool buttonDown = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool buttonLeft = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
        bool buttonRight = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            conveyor.disable_color_sensor();
        }
        bool buttonB = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        // Move the chassis with arcade drive
        chassis.arcade(leftY, rightX);
        // Control subsystems based on button inputs
        conveyor.control(buttonL1, buttonL2, buttonR1, buttonR2);
        goal_transfer.control(buttonR1, false);
        Lipper.control(buttonY, true);
        Descore.control(buttonDown,true);
        // Delay to save resources
        pros::delay(10);
    }
}