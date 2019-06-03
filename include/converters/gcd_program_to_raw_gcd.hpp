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

#ifndef __CONVERTERS_GCD_PROGRAM_TO_RAW_GCD_HPP___
#define __CONVERTERS_GCD_PROGRAM_TO_RAW_GCD_HPP___

#include <gcd/gcode_interpreter.hpp>
#include <hardware_dof_conf.hpp>
#include <functional>

namespace raspigcd {
namespace converters {

/**
 * converts simple gcd program into optimized and adapted program to the given machine
 * */
gcd::program_t program_to_raw_program (
    const gcd::program_t& prog_,
    const configuration::limits& conf_,
    const gcd::block_t& initial_state_ = {{'F',0}});

gcd::program_t program_to_raw_program_str (
    const std::string& prog_text_,
    const configuration::limits& conf_,
    const gcd::block_t& initial_state_ = {{'F',0}});

} // namespace converters
} // namespace raspigcd


#endif
