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



#include <converters/gcd_program_to_raw_gcd.hpp>
#include <movement/physics.hpp>
#include <movement/simple_steps.hpp>

#include <functional>

namespace raspigcd {
namespace converters {

gcd::program_t program_to_raw_program(
    const gcd::program_t& prog_,
    const configuration::limits& conf_,
    const gcd::block_t& initial_state_)
{
    using namespace gcd;
    block_t machine_state = merge_blocks({{'X', 0}, {'Y', 0}, {'Z', 0}, {'A', 0}, {'F', 0.0}}, initial_state_);
    partitioned_program_t prog_grouped = group_gcode_commands(prog_, machine_state);

    program_t ret = {};
    //std::cout << "ok, robim" << std::endl;
    for (auto& ppart : prog_grouped) {
        auto part_fixed = ppart;
        if (ppart.size() != 0) {
            //std::cout << "ppart" << std::endl;
            if (ppart[0].count('M') == 0) {
                //std::cout << "::G: " << (int)(ppart[0]['G']) << std::endl;
                switch ((int)(ppart[0]['G'])) {
                case 4: // no change
                    break;
                case 0:
                    //std::cout << "g0_move_to_g1_sequence" << std::endl;
                    part_fixed = g0_move_to_g1_sequence(ppart, conf_, machine_state);
                    //std::cout << "g0 -> g1: ";
                    // for (auto& ms : ppart) {
                    //     for (auto& s : ms) {
                    //         std::cout << s.first << ":" << s.second << " ";
                    //     }
                    //     std::cout << std::endl;
                    // }
                    break;
                case 1:
                    // part_fixed = g1_move_to_g1_with_machine_limits(ppart, cfg, machine_state);
                    break;
                }
            } else {
            }
        }
        //        for (auto& s : machine_state) {
        for (auto pp : part_fixed) {
            //for (auto& s : pp) {
            //    std::cout << s.first << ":" << s.second << " ";
            //}
            //std::cout << std::endl;
            machine_state = merge_blocks(machine_state, pp);
            machine_state.erase('G');
            machine_state.erase('M');
        }
        //std::cout << "  OK" << std::endl;
        ret.insert(ret.end(), part_fixed.begin(), part_fixed.end());
    }
    auto rret = ret;
    ret.clear();
    machine_state = merge_blocks({{'X', 0}, {'Y', 0}, {'Z', 0}, {'A', 0}, {'F', 0.0}}, initial_state_);
    for (auto e : rret) {
        auto ee = e;
        for (auto kv : ee) {
            if (machine_state.count(kv.first)) {
                if (kv.second == machine_state.at(kv.first)) {
                    if ((kv.first != 'G') && (kv.first != 'M')) e.erase(kv.first);
                }
            }
        }
        machine_state = merge_blocks(machine_state, e);
        ret.push_back(e);
    }
    return ret;
}

gcd::program_t program_to_raw_program_str(
    const std::string& prog_text_,
    const configuration::limits& conf_,
    const gcd::block_t& initial_state_)
{
    return program_to_raw_program(gcd::gcode_to_maps_of_arguments(prog_text_),
        conf_,
        initial_state_);
}

} // namespace converters
} // namespace raspigcd
