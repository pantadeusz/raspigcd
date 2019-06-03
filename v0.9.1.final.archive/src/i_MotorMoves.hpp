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

#ifndef __TP_MOTOR_I_MOTORMOVES_HPP__
#define __TP_MOTOR_I_MOTORMOVES_HPP__

#include "i_Stepper.hpp"
#include "i_Spindle.hpp"
#include "i_Buttons.hpp"
#include "dto_Steps.hpp"
#include "dto_MotorCommand.hpp"
#include "s_SafeQueue.hpp"

#include <array>
#include <iostream>
#include <vector>

#include <chrono>
#include <thread>

namespace tp {
namespace motor {

    class i_MotorMoves {
    protected:
    public:
        /*
	 * pushes one command to command queue
	 */
        virtual void push(const MotorCommand& command) = 0;
        /**
	 * waits until command queue is empty. WARNING: If queue is paused, then the behavior of this
	 * method is unspecified!
	 */
        virtual void wait_finished() = 0;
        /**
	 * returns number of steps from origin
	 */
        virtual Steps steps_from_origin() = 0;

        /**
	 * sets steps from origin
	 */
        virtual void steps_from_origin(Steps s) = 0;

        virtual std::array<unsigned char, 4> buttons_state() = 0;

        virtual bool is_steppers_enabled() = 0;

        virtual void clear_command_queue() = 0;
        /**
	 * resets origin to 0
	 */
        virtual void reset_origin() = 0;

        virtual int get_min_step_time_us() = 0;

        /**
	 * blocks or unblocks reading command queue.
	 * */
        virtual void set_paused(bool paused) = 0;

        /**
	 * checks if the queue is paused.
	 * */
        virtual bool is_paused() = 0;
    };

} // namespace motor
} // namespace tp

#endif
