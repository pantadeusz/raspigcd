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


#ifndef __RASPIGCD_MOVEMENT_SIMPLE_STEPS_T_HPP__
#define __RASPIGCD_MOVEMENT_SIMPLE_STEPS_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/stepping_commands.hpp>
#include <list>
#include <steps_t.hpp>

namespace raspigcd {
namespace movement {

/**
 * simple steps calculations, like removal of unnecesary steps (duplicate steps) and other similar tasks
 * */
namespace simple_steps {

/**
 * calculate the number of steps that must be performed to reach destination steps. Effectively checks the max steps count
 * */
int steps_remaining( const steps_t& steps_, const steps_t& destination_steps_ );


/**
 * remove unnecessary points from list of steps
 * */
hardware::multistep_commands_t collapse_repeated_steps(const std::list<hardware::multistep_command>& ret);



/**
 * @brief generates steps to reach given destination steps
 * @arg ret this is the container for steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
void chase_steps(hardware::multistep_commands_t &ret, const steps_t& start_pos_, const steps_t &destination_pos_);
//void chase_steps(std::list<hardware::multistep_command> &ret, const steps_t& start_pos_, steps_t destination_pos_);


/**
 * @brief generates steps to reach given destination steps
 * @arg steps_ current steps count
 * @arg destination_steps_ desired steps count
 */
hardware::multistep_commands_t chase_steps( const steps_t& steps_, const steps_t &destination_steps_ );


} // namespace simple_steps

} // namespace movement
} // namespace raspigcd

#endif
