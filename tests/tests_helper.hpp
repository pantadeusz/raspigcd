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


#ifndef __TESTS_HELPER__HPP___
#define __TESTS_HELPER__HPP___

#include <gcd/gcode_interpreter.hpp>

std::vector<std::vector<int>> simulate_moves_on_image(
    const raspigcd::gcd::program_t& prg, const raspigcd::gcd::block_t& initial_state = {});
std::vector<std::vector<int>> simulate_moves_on_image(
    const raspigcd::hardware::multistep_commands_t & prg,
    raspigcd::hardware::motor_layout &motor_layout
    );
int image_difference(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b);



std::vector<std::vector<int>> load_image(std::string filename);
void save_image(const std::string filename, const std::vector<std::vector<int>> &img_dta);

#endif