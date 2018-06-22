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

#ifndef __TP_MOTOR_I_STEPPER_HPP__
#define __TP_MOTOR_I_STEPPER_HPP__

#include <iostream>
#include <memory>
#include <vector>

namespace tp {

namespace motor {

    class i_Stepper {
    public:
        /**
	 * executes one motors step. takes value - number of steps to perform in given direction (positive or negative). 
	 * It will perform one step in given direction and reduce steps remaining by 1
	 */
        virtual void step(std::array<signed char, 4>& dir) = 0;
        /**
	 * enable or disable stepper motor
	 */
        virtual void enabled(bool en) = 0;

        virtual bool is_enabled() = 0;
    };

    typedef std::shared_ptr<i_Stepper> p_Stepper;

} // namespace motor
} // namespace tp

#endif
