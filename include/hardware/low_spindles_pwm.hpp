/*
    Raspberry Pi G-CODE interpreter

    Copyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl

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


#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL_SPINDLES_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL_SPINDLES_T_HPP__

#include <atomic>

namespace raspigcd {
namespace hardware {

class low_spindles_pwm
{
private:
public:

    /**
     * @brief Set the spindle pwm power 
     * 
     * @param i index of spindle (usualy 0)
     * @param v value between 0 (stop) and 1 (maximal speed)
     * @return raspberry_pi_3&  the reference to this object
     */
    virtual void spindle_pwm_power(const int i, const double v) = 0;
};

} // namespace hardware
} // namespace raspigcd

#endif
