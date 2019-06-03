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


#ifndef __RASPIGCD_MOTOR_LAYOUT_T_HPP__
#define __RASPIGCD_MOTOR_LAYOUT_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <memory>
#include <steps_t.hpp>

namespace raspigcd {
namespace hardware {
class motor_layout
{
private:
public:
    /**
         * @brief converts distances in milimeters to number of ticks
         */
    virtual steps_t cartesian_to_steps(const distance_t& distances_) = 0;
    /**
         * @brief converts number of ticks to distances in milimeters
         */
    virtual distance_t steps_to_cartesian(const steps_t& steps_) = 0;

    virtual void set_configuration(const configuration::actuators_organization& cfg) = 0;

    static std::shared_ptr<motor_layout> get_instance(const configuration::actuators_organization& cfg);
};
} // namespace hardware
} // namespace raspigcd

#endif
