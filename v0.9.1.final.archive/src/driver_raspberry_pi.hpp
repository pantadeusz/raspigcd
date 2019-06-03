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

#ifndef __TP_MOTOR_DRIVER_RASPBERRY_PI_HPP__
#define __TP_MOTOR_DRIVER_RASPBERRY_PI_HPP__ 1

#include "i_Buttons.hpp"
#include "i_Spindle.hpp"
#include "i_Stepper.hpp"

#include "dto_StepperPiConfig.hpp"
#include "dto_ButtonsPiConfig.hpp"
#include "dto_SpindlePiConfig.hpp"


namespace tp {
namespace motor {

    p_Stepper StepperPi_factory(const std::vector<StepperPiConfig> stc);
    p_Spindle SpindlePi_factory(const SpindlePiConfig stc);
    p_Buttons ButtonsPi_factory(const ButtonsPiConfig stc);
    // w_Stepper wStepperPi_factory(const StepperPiConfig stc);

} // namespace motor
} // namespace tp

#endif
