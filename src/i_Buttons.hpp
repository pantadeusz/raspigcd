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

#ifndef __TP_MOTOR_I_BUTTONS_HPP__
#define __TP_MOTOR_I_BUTTONS_HPP__ 1

#include <iostream>
#include <memory>
#include <vector>

namespace tp {

namespace motor {

    class i_Buttons {
    public:
        /**
	 * set the spindle speed: 0..1, where 0 means stopped, and 1 means CW rotation at maximum speed
	 */
        virtual std::array<unsigned char, 4> getButtons() = 0;
    };

    typedef std::shared_ptr<i_Buttons> p_Buttons;

} // namespace motor
} // namespace tp

#endif
