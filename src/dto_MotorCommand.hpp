#ifndef __TP_MOTOR_MOTORCOMMAND_HPP__
#define __TP_MOTOR_MOTORCOMMAND_HPP__

#include <array>
#include <iostream>
#include <vector>

#include <chrono>
#include <thread>

namespace tp {
namespace motor {

    class MotorCommand {
    public:
        enum Command {
            step,
            steppersOn,
            steppersOff,
            nop,
            halt,
            spindle
        };
        int delayBefore;
        std::array<signed char, 4> steps; // steps to perform
        char commands; // additional commands - turn on or off motors
    };

}}

#endif
