#pragma once

#include "pros/motor_group.hpp"
#include "lemlib/api.hpp"
#include "subsystem.hpp"

namespace ArmNamespace {

enum class State { DOWN, WAIT, UP, VERTICAL_UP, DESCORE_UP, SCORE_UP, AUTO_UP, IDLE };

class Arm : public subsystem<State> {
    public:
        Arm(std::shared_ptr<pros::Motor> motor, std::shared_ptr<pros::Rotation> rotation,
            lemlib::ControllerSettings armAngularController, lemlib::ControllerSettings armAngularControllerSmallAngle)
            : motor_(std::move(motor)),
              rotation_(std::move(rotation)),
              armAngularPID(armAngularController.kP, armAngularController.kI, armAngularController.kD,
                            armAngularController.windupRange, true),
              armAngularPIDSmallAngle(armAngularControllerSmallAngle.kP, armAngularControllerSmallAngle.kI,
                            armAngularControllerSmallAngle.kD, armAngularControllerSmallAngle.windupRange, true) {
            motor_->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
            motor_->tare_position();
            motor_->set_brake_mode(pros::MotorBrake::hold);
            rotation_->set_position(0);
            rotation_->reset();
        }

        ~Arm() override = default;

        // Controls the arm based on button inputs
        void control(bool button_1, bool button_2, bool button_3, bool button_4) { // R2 Y Right UP
            
            if (button_1) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::UP);
            }  else if (currState == State::UP) {
                moveToState(State::DOWN);
            }  

            if (button_2 && currState == State::DOWN) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::WAIT);
            } else if (button_2 && (currState == State::WAIT || currState == State::VERTICAL_UP)) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::SCORE_UP);
            } else if (button_2 && currState == State::SCORE_UP) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::DOWN);
            }

            if (button_3) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::VERTICAL_UP);
            }
            if (button_4) {
                motor_->set_brake_mode(pros::MotorBrake::hold);
                moveToState(State::DESCORE_UP);
            }
            
        }



        void changeVelocity(float ratio) { ratio_ = ratio; }

        bool getManual() { return manual_; }

        // Getter for rotation sensor
        std::shared_ptr<pros::Rotation> getRotation() const { return rotation_; }

        // Getter for the arm's state as a string
        std::string getState() const {
            switch (currState) {
                case State::UP: return "UP";
                case State::WAIT: return "WAIT";
                case State::DOWN: return "DOWN";
                case State::VERTICAL_UP: return "VERTICAL_UP";
                case State::SCORE_UP: return "SCORE_UP";
                default: return "UNKNOWN";
            }
        }

        void moveToState(State newState) {
            armAngularPID.reset();
            armAngularPIDSmallAngle.reset();
            currState = newState;
        }

        void moveToState(State newState, int time) {
            timer.set(time);
            moveToState(newState);
        }

        void resetRotation(float rotation) { rotation_->set_position(rotation); }
    private:
        std::shared_ptr<pros::Motor> motor_;
        std::shared_ptr<pros::Rotation> rotation_;
        float ratio_ = 1;
        bool manual_;
    protected:
        lemlib::Timer delay_timer {20};

        // angular motion controller
        lemlib::PID armAngularPID, armAngularPIDSmallAngle;

        // Task to control the arm's movement
        void runTask() override final {
            if (currState == State::DOWN) motor_->set_brake_mode(pros::MotorBrake::coast);
            else motor_->set_brake_mode(pros::MotorBrake::hold);

            switch (currState) {
                case State::UP:
                    if (rotation_->get_position() <= MAX_DEGREES) {
                        motor_->move(127 * ratio_);
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::WAIT:
                    if (float angularError = (WAIT_DEGREES - rotation_->get_position()) / 100.0;
                        abs(angularError) > 2) {
                        if (angularError < WAIT_DEGREES * 0) {
                            motor_->move(armAngularPIDSmallAngle.update(angularError));
                        } else {
                            motor_->move(armAngularPID.update(angularError));
                        }
                        //motor_->move(armAngularPIDSmallAngle.update(angularError));
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::DOWN:
                    if (rotation_->get_position() > DOWN_DEGREES) {
                        motor_->move(-127 * ratio_);
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::VERTICAL_UP:
                    if (float angularError = (VERTICAL_DEGREES - rotation_->get_position()) / 100.0;
                    abs(angularError) > 2) {
                        motor_->move(armAngularPID.update(angularError));
                    } else {
                        motor_->move(0);
                    }
                break;
                case State::DESCORE_UP:
                    if (float angularError = (DESCORE_DEGREES - rotation_->get_position()) / 100.0;
                    abs(angularError) > 2) {
                        motor_->move(armAngularPID.update(angularError));
                    } else {
                        motor_->move(0);
                    }
                break;
                case State::SCORE_UP:
                    if (float angularError = (SCORE_DEGREES - rotation_->get_position()) / 100.0;
                        abs(angularError) > 2) {
                        motor_->move(armAngularPID.update(angularError));
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::AUTO_UP:
                    if (float angularError = (AUTO_DEGREES - rotation_->get_position()) / 100.0;
                        abs(angularError) > 2) {
                        motor_->move(armAngularPID.update(angularError));
                    } else {
                        motor_->move(0);
                    }
                    break;
                case State::IDLE: motor_->move(0); break;
            }
        }
};
} // namespace ArmNamespace