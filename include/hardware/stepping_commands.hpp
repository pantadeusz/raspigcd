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


#ifndef __RASPIGCD_HARDWARE_STEPPING_COMMANDS_T_HPP__
#define __RASPIGCD_HARDWARE_STEPPING_COMMANDS_T_HPP__

#include <cstdint>
#include <vector>
#include <array>

#include <hardware_dof_conf.hpp>

namespace raspigcd {
namespace hardware {

struct single_step_command {
    unsigned char step : 1;
    unsigned char dir : 1;//, sync_laser_en: 1, sync_laser: 1;
};

struct multistep_command {
    std::array<single_step_command,4> b; // command that have to be executed synchronously
    int count;                                    // number of times to repeat the command, it means that the command will be executed repeat n.
};
using multistep_commands_t = std::vector<multistep_command>;

inline bool operator==(const single_step_command &a, const single_step_command &b) {
    return (a.step == b.step) && (a.dir == b.dir);
    //return *(char*)&a == *(char*)&b;
}

/**
 * @brief Compares the steps command. Ignore count
 */
inline bool multistep_command_same_command(const multistep_command &a, const multistep_command &b) {
    for (unsigned i = 0; i < a.b.size(); i++) if (!(a.b[i] == b.b[i])) return false;
    return true;
}

} // namespace hardware
} // namespace raspigcd

#endif
