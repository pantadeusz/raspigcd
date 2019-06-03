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


#include <gcd/remove_g92_from_gcode.hpp>
#include <movement/physics.hpp>
#include <movement/simple_steps.hpp>

#include <functional>

namespace raspigcd {
namespace gcd {

program_t remove_g92_from_gcode(const program_t& input_program_)
{
    program_t result;
    distance_t current_shift;
    block_t current_state;
    for (auto e : input_program_) {
        for (auto &[k,v]:e) {
            v = ((int)(v * 1024.0));
            v = v/1024.0;
        }
        if (e.count('G') && ((int)e.at('G') == (int)92)) {
            auto new_pos = block_to_distance_t(merge_blocks(current_state, e));
            auto old_pos = block_to_distance_t(current_state);
            current_shift = current_shift + new_pos-old_pos;
            current_state = merge_blocks(current_state, e);
        } else {
            if (e.count('X')) e['X'] = e['X'] - current_shift[0];
            if (e.count('Y')) e['Y'] = e['Y'] - current_shift[1];
            if (e.count('Z')) e['Z'] = e['Z'] - current_shift[2];
            if (e.count('A')) e['A'] = e['A'] - current_shift[3];
            current_state = merge_blocks(current_state, e);
            result.push_back(e);
        }
        //std::cout << "shift: " << current_shift << std::endl;
        current_state.erase('G');
        current_state.erase('M');
    }
    return result;
}

// sgcd::program_t
} // namespace gcd
} // namespace raspigcd
