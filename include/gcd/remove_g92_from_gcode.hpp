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


#ifndef __CONVERTERS_REMOVE_G4_FROM_GCODE_HPP___
#define __CONVERTERS_REMOVE_G4_FROM_GCODE_HPP___

#include <functional>
#include <gcd/gcode_interpreter.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware_dof_conf.hpp>

namespace raspigcd {
namespace gcd {

program_t remove_g92_from_gcode(const program_t &input_program_);

}
} // namespace raspigcd

#endif
