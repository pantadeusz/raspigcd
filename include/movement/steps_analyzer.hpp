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


#ifndef __RASPIGCD_MOVEMENT_STEPS_ANALYZER_T_HPP__
#define __RASPIGCD_MOVEMENT_STEPS_ANALYZER_T_HPP__

#include <cmath>
#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping_commands.hpp>
#include <memory>
#include <movement/simple_steps.hpp>
#include <steps_t.hpp>
#include <variant>
#include <functional>


namespace raspigcd {
namespace movement {

class steps_analyzer
{
protected:
    hardware::motor_layout* _motor_layout;
    std::shared_ptr<hardware::motor_layout> _motor_layout_ptr;

public:
    void set_motor_layout(std::shared_ptr<hardware::motor_layout> ml) {
        _motor_layout = ml.get();
        _motor_layout_ptr = ml;
    };

    steps_analyzer(std::shared_ptr<hardware::motor_layout> ml) {
        set_motor_layout(ml);
    };

    /**
     * @brief Returns the position in steps _after_ the execution of given numbers of tick.
     * 
     * Note that if the tick number is 0  then no ticks are executed.
     * 
     */
    steps_t steps_from_tick(const hardware::multistep_commands_t &commands_to_do,const int tick_number) const ;

    int get_last_tick_index(const hardware::multistep_commands_t &commands_to_do) const ;


};

} // namespace movement
} // namespace raspigcd

#endif
