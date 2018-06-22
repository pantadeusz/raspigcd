/*

    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/

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
