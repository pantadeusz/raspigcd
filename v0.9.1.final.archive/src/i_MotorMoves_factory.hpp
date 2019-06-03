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

#ifndef __TP_MOTOR_MOTORMOVES_FACTORY_HPP__
#define __TP_MOTOR_MOTORMOVES_FACTORY_HPP__

#include "i_Stepper.hpp"
#include "i_Spindle.hpp"
#include "i_Buttons.hpp"

#include "dto_Steps.hpp"
#include "dto_MotorCommand.hpp"
#include "s_SafeQueue.hpp"
#include "i_MotorMoves.hpp"

#include <array>
#include <iostream>
#include <vector>

#include <chrono>
#include <thread>

namespace tp {
namespace motor {

    std::shared_ptr<i_MotorMoves> MotorMoves_factory(i_Stepper* motors_, i_Spindle* spindle_, i_Buttons* buttons_, int minStepTime_);
    std::shared_ptr<i_MotorMoves> MotorMoves_factory(p_Stepper motors_, p_Spindle spindle_, p_Buttons buttons_, int minStepTime_);

} // namespace motor
} // namespace tp

#endif
