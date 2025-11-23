#pragma once

#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "constants.hpp"

template <typename StateType> class subsystem {
    public:
        subsystem() {
            task = std::make_unique<pros::Task>([this] { taskRunner(); });
            task->resume();
        }

        virtual ~subsystem() {
            if (task) { task->remove(); }
        }

        void moveToState(StateType newState) { currState = newState; }

        void moveToState(StateType newState, int time) {
            timer.set(time);
            moveToState(newState);
        }

        StateType getState() { return currState; }
    protected:
        // Function that will be overridden by derived classes to provide custom task functionality
        virtual void runTask() = 0;
        StateType currState;
        lemlib::Timer timer {0};
    private:
        std::unique_ptr<pros::Task> task; // Use a unique_ptr to manage the task

        // Internal task function that runs in the base class
        void taskRunner() {
            while (true) {
                runTask(); // Call the derived class's task logic
                pros::delay(10); // Adjust the delay as necessary
            }
        }
};
