#pragma once
#include "pros/adi.hpp"
#include "subsystems/subsystem.hpp"

namespace HolderNamespace {
enum class State { RELEASE, HOLD };

class Holder : public subsystem<HolderNamespace::State> {
    public:
        Holder(std::shared_ptr<pros::adi::DigitalOut> clamp, std::shared_ptr<pros::Distance> distance)
            : clamp_(std::move(clamp)),
              distance_(std::move(distance)) {}

        Holder(std::shared_ptr<pros::adi::DigitalOut> clamp)
            : clamp_(std::move(clamp)) {}

        // Defaulted destructor as the base class handles task cleanup
        ~Holder() override = default;

        // Toggles state based on button press
        void control(bool button, bool isToggle) {
            if (isToggle) {
                if (button) {
                    moveToState(currState == HolderNamespace::State::HOLD ? HolderNamespace::State::RELEASE
                                                                        : HolderNamespace::State::HOLD);
                    pros::delay(100);
                }
            } else {
                if (button) {
                    moveToState(HolderNamespace::State::HOLD);
                } else {
                    moveToState(HolderNamespace::State::RELEASE);
                }
            }

        }


        void deactivateAuto() { distance_ = nullptr; }

        bool detectedMogo() {
            return distance_ != nullptr and distance_->get_distance() < 100 and
                   currState != HolderNamespace::State::HOLD;
        }
    private:
        std::shared_ptr<pros::adi::DigitalOut> clamp_;
        std::shared_ptr<pros::Distance> distance_ = nullptr;

        // Run the task according to the current state
        void runTask() override final {
            // if (detectedMogo() and timer.isDone()) {
            //     currState = HolderNamespace::State::HOLD;
            //     timer.set(AUTO_HOLD_TIMEOUT);
            // };

            clamp_->set_value(currState == HolderNamespace::State::HOLD ? 100 : 0);
        }
};
} // namespace HolderNamespace