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


#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL_TIMERS_BUSY_WAIT_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL_TIMERS_BUSY_WAIT_T_HPP__

#include <hardware/low_timers.hpp>
#include <functional>

namespace raspigcd {
namespace hardware {
namespace driver {


class low_timers_busy_wait : public low_timers {
private:
public:
   
    /**
     * @brief start the timer
     * 
     */
    std::chrono::high_resolution_clock::time_point start_timing();

    /**
     * @brief wait for the tick to end.
     * Remember to run start_timing first!
     */
    std::chrono::high_resolution_clock::time_point wait_for_tick_us(
        const std::chrono::high_resolution_clock::time_point &prev_timer,
        const int64_t t);
};

}
} // namespace hardware
} // namespace raspigcd

#endif
