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

#ifndef __TP_MOTOR_MACHINE_HPP__
#define __TP_MOTOR_MACHINE_HPP__

#include "i_Machine.hpp"
#include "i_CoordTranslate.hpp"
#include "i_MotorMoves_factory.hpp"
#include <atomic>

namespace tp {
namespace motor {
    using tp::coord::Position;


    class Machine : public i_Machine {
    protected:
        std::shared_ptr<tp::coord::i_CoordTranslate> coordTranslator_;
        std::shared_ptr<i_MotorMoves> motorMoves_;
        Position minimalStepMm; // this is the positive vector for minimal step in direction (1,1,1,1) in steps

        std::mutex m_csteps_;
        Steps currentSteps_; // current position in steps

        std::atomic_bool doBreak;

    public:
        void breakExecution();

        // goes to selected position
        void gotoXYZ(const Position& pos, double v, int selectedAxes = 0x0ff, double maxVelocityNoAccel = -1, double ddt = 100);

        // enables or disables spindle
        void spindleEnabled(bool _enabled);

        // set steppers on or off
        void steppersEnable(bool _enabled);

        bool isSteppersEnabled();

        // pause for given time
        void pause(int ms_time);

        // resets the position
        void setPosition(const Position& pos);
        // returns the current position
        Position getPosition(bool sync = true);

        // waits for commands to finish
        void waitFinish();
        // configuration of hardware layer
        void setMotorMoves(std::shared_ptr<i_MotorMoves> _motorMoves);
        // gets the coordinate system translation -- this will make smart pointers out of it
        void setCoordinateSystem(std::shared_ptr<tp::coord::i_CoordTranslate> _coordTranslator);

        int waitForEndstopTrigger();

        std::array<unsigned char, 4> getEndstops();

        Machine() { doBreak = false; }
    };

} // namespace motor
} // namespace tp

#endif
