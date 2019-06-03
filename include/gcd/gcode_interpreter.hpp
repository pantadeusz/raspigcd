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


#ifndef __RASPIGCD_GCD_GCODE_INTERPRETER_HPP__
#define __RASPIGCD_GCD_GCODE_INTERPRETER_HPP__

#include <configuration.hpp>
//#include <memory>
//#include <hardware/low_steppers.hpp>
#include <hardware/motor_layout.hpp>
//#include <hardware/stepping.hpp>
//#include <gcd/factory.hpp>
//#include <movement/path_intent_t.hpp>

#include <movement/physics.hpp>

#include <list>
#include <map>
#include <string>


namespace raspigcd {
namespace gcd {

using block_t = std::map<char, double>;             // represents N001G0X10Y20
using program_t = std::vector<block_t>;               // represents whole program without empty lines
using partitioned_program_t = std::vector<program_t>; // represents program partitioned into different sections for optimization and interpretation

partitioned_program_t insert_additional_nodes_inbetween(partitioned_program_t &partitioned_program_, const block_t &initial_state, const configuration::limits &machine_limits);

/**
 * @brief removed duplicate blocks. If the feedrate is different, then it interprets it as rapid velocity shift and does not remove
 */
program_t remove_duplicate_blocks(const program_t& program_states, const block_t &initial_state);

/**
 * @brief calculates last state after execution of the program.
 */
block_t last_state_after_program_execution(const program_t &program_, const block_t &initial_state_);

/**
 * convert block to position
 * @untested
 * */
distance_t block_to_distance_t(const block_t& block);

/**
 * calculates movement vector from block a to block b
 * @untested
 * */
distance_t blocks_to_vector_move(const block_t& block_a, const block_t& block_b);

/**
 * @brief converts distance to the block of X..Y..Z..A..
 * @untested
 */
block_t distance_to_block(const distance_t& dist);



//movement::path_intent_t generate_path_intent(const program_t& parsed_program_);

/**
 * @brief  generates coordinates from block
 * 
 */
distance_t block_to_distance_t(const block_t & block);

/**
 * @brief returns the movement vector out of the two consecutive blocks
 */
distance_t blocks_to_vector_move(const block_t & block_a, const block_t & block_b);


/**
 * @brief Generates string based on gcode grouped by fragments G1, G0 and M
 */
std::string back_to_gcode(const partitioned_program_t &btg);

//partitioned_program_t apply_machine_limits(
//    const partitioned_program_t& program_states,
//    // limits: acceleration, max_velocity, min_velocity_no_accel
//    const block_t & initial_state = {{'F',1}} );

// program_t apply_machine_limits (const program_t& program_states,
//                const configuration::limits &machine_limits);


/**
 * @brief Adds limits to the machine turns based on the maximal speeds and angles
 * the states receives the minimum of feedrate based on intended feedrate and the
 * maximal feedrate based on turn angle and limits.
 * 
 * The first and last G1 command is interpreted that it is on the 90deg turn.
 */
program_t apply_limits_for_turns (const program_t& program_states,
                const configuration::limits &machine_limits);



program_t g1_move_to_g1_with_machine_limits(const program_t& program_states,
    const configuration::limits& machine_limits,
    block_t current_state = {{'X',0},{'Y',0},{'Z',0},{'A',0}});

/**
 * @brief converts G0 into sequences of G1 moves that accelerates to maximal
 * speed, then move with constant speed, and then decelerates to minimal speed.
 */
program_t g0_move_to_g1_sequence (const program_t& program_states,
                const configuration::limits &machine_limits,
                block_t current_state = {{'X',0},{'Y',0},{'Z',0},{'A',0}});



/**
 * @brief Gropus gcode commands so the other parts can focus on the simple interpretation of parts
 * 
 * It groups into G0, G1 and M codes.
 * 
 */
partitioned_program_t group_gcode_commands(const program_t& program_states, const block_t & initial_state = {{'F',1}} );

/**
 * @brief updates values in destination block from source block.
 * 
 * For example, in destination there is X10 Y20, and in source is Y1 Z0, then
 * the result will be X10 Y1 Z0
 */
block_t merge_blocks(const block_t &destination, const block_t &source);

/**
 * @brief reduces gcode that only difference is left
 * 
 * @untested
 */
block_t diff_blocks(const block_t& destination, const block_t& source);

/**
 * @brief Interprets gcode program to list of gcode state updates
 * 
 * for example, for
 *   G0X10
 *   G1Y2
 * will give
 *   {
 *     {{'G',0},{'X',10}},
 *     {{'G',1},{'Y',2}}
 *   }
 */
program_t gcode_to_maps_of_arguments(const std::string& program_);

block_t command_to_map_of_arguments(const std::string& command_);

/**
 * @brief UNTESTED: optimizes program using douglas + puecker algorithm
 * 
 */
program_t optimize_path_douglas_peucker(const program_t &program_, 
    const double epsilon = 0.0125, 
    const block_t &initial_state = {{'X',0},{'Y',0},{'Z', 0},{'F',0.1}});



auto linear_interpolation = [](auto x, auto x0, auto y0, auto x1, auto y1) {
    return y0 * (1 - (x - x0) / (x1 - x0)) + y1 * ((x - x0) / (x1 - x0)); // percentage of the max_no_accel_speed
};









/// UNTESTED
distance_t block_to_distance_t(const block_t& block);
distance_with_velocity_t block_to_distance_with_v_t(const block_t& block);
block_t distance_to_block(const distance_t& dist);
block_t distance_with_velocity_to_block(const distance_t& dist);
distance_t blocks_to_vector_move(const block_t& block_a, const block_t& block_b);


} // namespace gcd
} // namespace raspigcd

#endif
