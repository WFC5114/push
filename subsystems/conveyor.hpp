#pragma once
#include "lemlib/timer.hpp"
#include "spinner.hpp"
#include <cmath>
#include "pros/optical.hpp"
#include "holder.hpp"
#include "subsystem.hpp"

namespace ConveyorNamespace {

enum class State { STOP, INTAKE, OUTTAKE, MIDDLE_GOAL, HIGH_GOAL};

enum Color { RED = 0, BLUE = 1 };

class Conveyor : public subsystem<ConveyorNamespace::State> {
    public:
        Conveyor(SpinnerNamespace::Spinner* intake, SpinnerNamespace::Spinner* hood,
                 std::shared_ptr<pros::Optical> optical_sensor)
            : intake_(std::move(intake)),
              hood_(std::move(hood)),
              optical_sensor_(std::move(optical_sensor)), MiddleGoalTimer(0) {}

        ~Conveyor() override = default;

        void setInitColor(const Color color) { init_color_ = color; }

        // Control conveyor direction based on buttons
        void control(bool button_intake, bool button_outtake, bool button_middle_goal, bool button_high_goal) {
            if (button_intake) currState = ConveyorNamespace::State::INTAKE;
            else if (button_outtake) currState = ConveyorNamespace::State::OUTTAKE;
            else if (button_middle_goal) currState = ConveyorNamespace::State::MIDDLE_GOAL;
            else if (button_high_goal) currState = ConveyorNamespace::State::HIGH_GOAL;
            else currState = ConveyorNamespace::State::STOP;

            moveToState(currState);
        }


        // Check if a wrong ring is detected based on hue and proximity
        bool detectWrongRing() {
        if (!optical_sensor_) return false;  
        int hue = optical_sensor_->get_hue();
        if (hue == PROS_ERR_F) return false; 

        int proximity = optical_sensor_->get_proximity();
        if (proximity < PROXIMITY_THRESHOLD) return false;

        switch (this->init_color_) {
            case RED:
                return (hue > BLUE_THRESHOLD && hue < BLUE_HIGH_THRESHOLD);
            case BLUE:
                return (hue < RED_THRESHOLD || hue > RED_HIGH_THRESHOLD);
            default:
                return false;
            }
        }

        void highGoalWithSensor(int duration_time, int reverse_time) {
            auto startTime = pros::millis();
            enable_color_sensor_ = true;
            bool wrongColorDetected = false;

            moveToState(ConveyorNamespace::State::HIGH_GOAL);
            while (pros::millis() - startTime < duration_time) {
                if (enable_color_sensor_ && detectWrongRing()) {
                    wrongColorDetected = true;

                    intake_->moveToState(SpinnerNamespace::State::BACKWARD);
                    hood_->moveToState(SpinnerNamespace::State::BACKWARD);
                    pros::delay(reverse_time);
                    moveToState(ConveyorNamespace::State::STOP);
                    break; 
                }
                pros::delay(check_interval);
                }
           if (!wrongColorDetected) {
            moveToState(ConveyorNamespace::State::STOP);
            }
        }
        
        // Get the initial wrong hue value
        Color getInitColor() const { return init_color_; }
        bool isColorSensorEnabled() const { return enable_color_sensor_; }

        SpinnerNamespace::Spinner* getIntake() { return intake_; }

        SpinnerNamespace::Spinner* getHood() { return hood_; }

        // Get the optical sensor object
        std::shared_ptr<pros::Optical> getOpticalSensor() { return optical_sensor_; }

        // Disable color sensor
        void disable_color_sensor() { enable_color_sensor_ = false; }

        // Enable color sensor
        void enable_color_sensor() { enable_color_sensor_ = true; }

        bool is_reversed() { return reverse_ ? true : false; }

    private:
        SpinnerNamespace::Spinner* intake_;
        SpinnerNamespace::Spinner* hood_;
        std::shared_ptr<pros::Optical> optical_sensor_ = nullptr;
        std::shared_ptr<pros::Distance> distance_ = nullptr;
        Color init_color_;
        bool enable_color_sensor_ = false;
        bool reverse_ = false;
        lemlib::Timer MiddleGoalTimer;
        ConveyorNamespace::State lastState_ = ConveyorNamespace::State::STOP;
        
        // Task that runs based on the current state
        void runTask() override final {
        // Check for state change to reset timer for MIDDLE_GOAL
        if (currState != lastState_) {
            if (currState == ConveyorNamespace::State::MIDDLE_GOAL) {
                MiddleGoalTimer.reset(); // start timer when entering MIDDLE_GOAL state
            }
            lastState_ = currState; // update lastState_ to current state
         }

        switch (currState) {
            case ConveyorNamespace::State::INTAKE:
                intake_->moveToState(SpinnerNamespace::State::FORWARD);
                hood_->moveToState(SpinnerNamespace::State::SLOW_BACKWARD);
                break;
            case ConveyorNamespace::State::OUTTAKE:
                intake_->moveToState(SpinnerNamespace::State::BACKWARD);
                hood_->moveToState(SpinnerNamespace::State::BACKWARD);
                break;
            case ConveyorNamespace::State::MIDDLE_GOAL:
                if(MiddleGoalTimer.getTimePassed() < 100) break;
                intake_->moveToState(SpinnerNamespace::State::Medium_Forward);
                hood_->moveToState(SpinnerNamespace::State::Medium_Forward);
                break;
            case ConveyorNamespace::State::HIGH_GOAL:
                intake_->moveToState(SpinnerNamespace::State::FORWARD);
                hood_->moveToState(SpinnerNamespace::State::FORWARD);
                

                break;
            case ConveyorNamespace::State::STOP:
                intake_->moveToState(SpinnerNamespace::State::IDLE);
                hood_->moveToState(SpinnerNamespace::State::SLOW_BACKWARD);
                break;
        }
            
            if (timer.isDone() and timer.getTimeSet() > 0) {
                moveToState(ConveyorNamespace::State::STOP);
                timer.set(0);
            }

      
        }

};
} // namespace ConveyorNamespace