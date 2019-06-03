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


#ifndef __RASPIGCD_HARDWARE_LOW_LEVEL__TIMERS_T_HPP__
#define __RASPIGCD_HARDWARE_LOW_LEVEL__TIMERS_T_HPP__

#include <chrono>

namespace raspigcd {
namespace hardware {


class low_timers
{
private:
public:
    /**
     * @brief delay in microseconds (1/1000000 s)
     * 
     * Internally it calls wait_for_tick_us(start_timing(),t);
     * 
     * @param t delay to wait. 
     */
    inline void wait_us(const int64_t t) {
        wait_for_tick_us(start_timing(),t);
    }

    /**
     * @brief start the timer
     * It actually takes the current time point.
     */
    virtual std::chrono::high_resolution_clock::time_point start_timing() = 0;

    /**
     * @brief wait for the tick to end. Time is microseconds! (1/1000000 s)
     * Remember to run start_timing first.
     * 
     * @param prev_timer
     * @param t next tick time is prev_timer + t. It tells us when to stop waiting
     */
    virtual std::chrono::high_resolution_clock::time_point wait_for_tick_us(
        const std::chrono::high_resolution_clock::time_point &prev_timer,
        const int64_t t) = 0;
};

} // namespace hardware
} // namespace raspigcd

#endif
