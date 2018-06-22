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

#ifndef __TP_MOTOR_I_MACHINE_HPP____
#define __TP_MOTOR_I_MACHINE_HPP____

#include "i_CoordTranslate.hpp"
#include "i_MotorMoves_factory.hpp"
#include <atomic>

namespace tp {
namespace motor {
    using tp::coord::Position;

    class i_Machine {
    public:
        class BreakException {
        public:
            std::string why;
            BreakException(const std::string& w = "stopped")
            {
                why = w;
            }
        };
        virtual void breakExecution() = 0;

        // goes to selected position
        virtual void gotoXYZ(const Position& pos, double v, int selectedAxes = 0x0ff, double maxVelocityNoAccel = -1, double ddt = 100) = 0;

        // enables or disables spindle
        virtual void spindleEnabled(bool _enabled) = 0;

        // set steppers on or off
        virtual void steppersEnable(bool _enabled) = 0;

        // get steppers state
        virtual bool isSteppersEnabled() = 0;

        // pause for given time
        virtual void pause(int ms_time) = 0;

        // resets the position
        virtual void setPosition(const Position& pos) = 0;
        // returns the current position
        virtual Position getPosition(bool sync = true) = 0;

        virtual int waitForEndstopTrigger() = 0;

        virtual std::array<unsigned char, 4> getEndstops() = 0;

        // waits for commands to finish
        virtual void waitFinish() = 0;
    };


} // namespace motor
} // namespace tp

#endif
