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

#include "s_Machine.hpp"

#include <chrono>
#include <cmath>

namespace tp {
namespace motor {
    using namespace tp::coord;

    //std::mutex Machine::m_csteps_;

    void Machine::breakExecution()
    {
        doBreak = true;
    }

    void Machine::gotoXYZ(const Position& pos0, double velocity, int selectedAxes, double maxVelocityNoAccel, double ddt)
    {
        auto pos = coordTranslator_.get()->translate(currentSteps_);
        if (maxVelocityNoAccel < 0)
            maxVelocityNoAccel = velocity;
        if (selectedAxes & 0x01) {
            pos[0] = pos0[0];
        }
        if (selectedAxes & 0x02) {
            pos[1] = pos0[1];
        }
        if (selectedAxes & 0x04) {
            pos[2] = pos0[2];
        }
        if (selectedAxes & 0x08) {
            pos[3] = pos0[3];
        }
        //auto pos = coordTranslator_.get()->translate( destSteps );
        auto destSteps = coordTranslator_.get()->translate(pos);
        std::unique_lock<std::mutex> lock_guard_steps(m_csteps_);

        auto p0 = coordTranslator_.get()->translate(currentSteps_);
        auto p1 = pos0;
        auto destinationSteps = coordTranslator_.get()->translate(pos0);
        
        auto dp = pos - p0;
        double l = ::sqrt(len2(dp)); // length in mm
        if (l <= 0) return;
        static auto max_fabs = [](double a, double b) {
            a = a < 0 ? -a : a;
            b = b < 0 ? -b : b;
            return a > b ? a : b;
        };

        //auto max_steps =
        //double dt_us = t_us/max_steps;

        // currentSteps_
        auto startSteps = currentSteps_;
        double delay_us = 10; // TODO: fix! 
        double dt = delay_us*0.000001; // TODO: fix!
                
        int stepsAccelerating = 0;
        double acceleration_length = -1;
        double v = (maxVelocityNoAccel>=velocity)?velocity:maxVelocityNoAccel; // current velocity
        double dv = ddt;
        int stage = 0; // 0 - accel, 1 - const, 2 - break
        Position vp = dp/l; // movement vector
        double delay_us_todo = 0;
        
        for (double l_done = 0; ::sqrt(len2(coordTranslator_.get()->translate(currentSteps_)-p0)) < l; ) {
            delay_us_todo += delay_us;
            //std::cout << "p = " << pos[0] << ", " << pos[1] << "v = " << v << "  dv = " << dv  << ";  l=" << l << " " << l_done << std::endl;
            switch (stage)
            {
                case 0:
                    if (v >= velocity) {
                        stage = 1;
                        acceleration_length = l_done;
                        dv = 0; // constant speed
                    }
                    break;
                case 1:
                    if ((l-l_done) <= acceleration_length) {
                        dv = -ddt;
                        stage = 2;
                    }
                    break;
                case 2:
                    if (v <= 0.001) {
                        v = 0.001;
                    }
                    break;
            }
            v = v + dv*dt;
            auto dl = v*dt+(dv*dt*dt)/2;
            if (v < 0.01) v = 0.01;
            if (v > velocity) v = velocity;
            l_done += dl;
            auto new_pos = pos + vp*dl;
            auto dSteps = coordTranslator_.get()->translate(new_pos) - coordTranslator_.get()->translate(pos);
            pos = new_pos;
            while ((dSteps[0] != 0) || (dSteps[1] != 0) || (dSteps[2] != 0) || (dSteps[3] != 0)) {
                MotorCommand cmd;
                if (delay_us_todo <= delay_us) delay_us_todo = delay_us;
                cmd.delayBefore = delay_us_todo;
                //std::cout << "   .. " << delay_us << "us" << std::endl;
                for (int i = 0; i < 4; i++) {
                    if (dSteps[i] > 0) {
                        cmd.steps[i] = 1;
                        dSteps[i]--;
                        currentSteps_[i] += 1;
                    } else if (dSteps[i] < 0) {
                        cmd.steps[i] = -1;
                        dSteps[i]++;
                        currentSteps_[i] -= 1;
                    } else
                        cmd.steps[i] = 0;
                }
                cmd.commands = MotorCommand::Command::step;
                if (doBreak) {
                    std::cout << "break execution. Throwing BreakException!" << std::endl;
                    motorMoves_.get()->clear_command_queue();
                    doBreak = false;
                    throw BreakException();
                }
                motorMoves_.get()->push(cmd);
                delay_us_todo = 0;
            }
        }

    }

    void Machine::spindleEnabled(bool _enabled)
    {
        MotorCommand cmd;
        cmd.delayBefore = 200;
        cmd.steps[0] = _enabled ? 127 : 0;
        cmd.commands = MotorCommand::Command::spindle; // _enabled?MotorCommand::Command::spindleOn:MotorCommand::Command::spindleOff;
        motorMoves_.get()->push(cmd);
    }

    void Machine::pause(int ms_time)
    {
        MotorCommand cmd;
        cmd.delayBefore = ms_time * 1000; // miliseconds time
        cmd.commands = MotorCommand::Command::nop;
        motorMoves_.get()->push(cmd);
    }
    // resets the position
    void Machine::setPosition(const Position& pos)
    {
        std::unique_lock<std::mutex>(m_csteps_);
        waitFinish();
        motorMoves_.get()->steps_from_origin(currentSteps_ = coordTranslator_.get()->translate(pos));
    }
    // returns the current position
    Position Machine::getPosition(bool sync_)
    {
        std::unique_lock<std::mutex>(m_csteps_);
        if (sync_)
            waitFinish();
        currentSteps_ = motorMoves_.get()->steps_from_origin();
        return coordTranslator_.get()->translate(currentSteps_);
    }

    void Machine::steppersEnable(bool _enabled)
    {
        MotorCommand cmd;
        cmd.delayBefore = 2000;
        cmd.commands = (_enabled) ? MotorCommand::Command::steppersOn : MotorCommand::Command::steppersOff;
        motorMoves_.get()->push(cmd);
    }

    bool Machine::isSteppersEnabled()
    {
        return motorMoves_.get()->is_steppers_enabled();
    }

    // waits for commands to finish
    void Machine::waitFinish()
    {
        motorMoves_.get()->wait_finished();
    }
    // configuration of hardware layer
    void Machine::setMotorMoves(std::shared_ptr<i_MotorMoves> _motorMoves)
    {
        motorMoves_ = _motorMoves;
        getPosition();
    }
    // gets the coordinate system translation -- this will make smart pointers out of it
    void Machine::setCoordinateSystem(std::shared_ptr<tp::coord::i_CoordTranslate> _coordTranslator)
    {
        coordTranslator_ = _coordTranslator;

        minimalStepMm = coordTranslator_.get()->translate(Steps(1, 1, 1, 1));
    }

    int Machine::waitForEndstopTrigger()
    {
        std::unique_lock<std::mutex>(m_csteps_);
        waitFinish(); // this should be done in sync
        auto prevState = motorMoves_.get()->buttons_state();
        while (true) {
            auto state = motorMoves_.get()->buttons_state();
            for (unsigned i = 0; i < state.size(); i++) {
                if (state[i] < prevState[i])
                    return i;
            }
            prevState = state;

            if (doBreak) {
                //std::cout << "break execution. Throwing BreakException!" << std::endl;
                doBreak = false;
                throw BreakException();
            }
            std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::microseconds(500));
        }
        return 0;
    }

    std::array<unsigned char, 4> Machine::getEndstops()
    {
        std::unique_lock<std::mutex>(m_csteps_);
        return motorMoves_.get()->buttons_state();
    }

} // namespace motor
} // namespace tp
