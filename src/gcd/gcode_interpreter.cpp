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


#include <gcd/gcode_interpreter.hpp>

//#include <memory>
//#include <hardware/low_steppers.hpp>
//#include <hardware/motor_layout.hpp>
//#include <hardware/stepping.hpp>
//#include <gcd/factory.hpp>
//#include <movement/path_intent_t.hpp>
#include <cmath>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <stdexcept>
#include <string>

namespace raspigcd {
namespace gcd {

/// UNTESTED
distance_t block_to_distance_t(const block_t& block)
{
    auto blk = block;
    return {(double)blk['X'], (double)blk['Y'], (double)blk['Z']};
}


distance_with_velocity_t block_to_distance_with_v_t(const block_t& block)
{
    distance_with_velocity_t ret = {};
    if (block.count('X')) ret[0] = block.at('X');
    if (block.count('Y')) ret[1] = block.at('Y');
    if (block.count('Z')) ret[2] = block.at('Z');
    ret[3] = 0;
    if (block.count('F'))
        ret.back() = block.at('F');
    else {
        ret.back() = 0.1;
        std::cerr << "WARNING: Feedrate is 0 in distance_with_velocity_t block_to_distance_with_v_t" << std::endl;
    }
    return ret;
}

/// UNTESTED
block_t distance_to_block(const distance_t& dist)
{
    return {
        {'X', dist[0]},
        {'Y', dist[1]},
        {'Z', dist[2]},
    };
}

/// UNTESTED
block_t distance_with_velocity_to_block(const distance_t& dist)
{
    return {
        {'X', dist[0]},
        {'Y', dist[1]},
        {'Z', dist[2]},
        {'F', dist.back()},
    };
}


/// UNTESTED
distance_t blocks_to_vector_move(const block_t& block_a, const block_t& block_b)
{
    return block_to_distance_t(block_b) - block_to_distance_t(block_a);
}


block_t last_state_after_program_execution(const program_t& program_, const block_t& initial_state_)
{
    block_t result = initial_state_;
    result['X'];
    result['Y'];
    result['Z'];
    for (auto e : program_) {
        if (result.count('M')) result.erase('M');
        if (e.count('G')) {
            if ((int)(e.at('G')) == 4) {
                e.clear(); // G4 means dwell, we don't need that
            }
        }
        result = merge_blocks(result, e);
    }
    return result;
}


partitioned_program_t insert_additional_nodes_inbetween(partitioned_program_t& partitioned_program_, const block_t& initial_state, const configuration::limits& machine_limits)
{
    using namespace raspigcd::movement::physics;
    partitioned_program_t ret;
    auto current_state = merge_blocks({{'X', 0.0}, {'Y', 0.0}, {'Z', 0.0}, {'A', 0.0}, {'F', 0.1}}, initial_state);
    for (const auto& subprogram : partitioned_program_) {
        if (subprogram.size() > 0) {
            if (subprogram[0].count('G')) {
                program_t nsubprog;
                nsubprog.reserve(subprogram.size() * 2);
                for (const auto& block : subprogram) {
                    //std::cout << "###############  G: " << block.at('G') << " ; state: "  << current_state['G'] << std::endl;
                    if ((block.at('G') == 0) || (block.at('G') == 1)) {
                        auto next_state = merge_blocks(current_state, block);
                        distance_t move_vec = blocks_to_vector_move(current_state, next_state);
                        if (move_vec.length() < 0.01) {
                            nsubprog.push_back(block);
                        } else {
                            double max_accel = machine_limits.proportional_max_accelerations_mm_s2(move_vec);
                            double max_no_acc_v = machine_limits.proportional_max_no_accel_velocity_mm_s(move_vec);
                            path_node_t a = {block_to_distance_t(current_state), max_no_acc_v}; //current_state['F']};
                            path_node_t b = {block_to_distance_t(next_state), next_state['F']};
                            path_node_t transition_point = calculate_transition_point(
                                a,
                                b,
                                max_accel);
                            move_vec = move_vec * 0.5;
                            if ((transition_point.p - a.p).length() < move_vec.length()) {
                                //std::cout << "++ a.p " << a.p << "  transition_point.p " << transition_point.p << "   b.p " << b.p << std::endl;
                                auto nmvect = (move_vec / move_vec.length()) * (transition_point.p - a.p).length();
                                auto mid_state_a = merge_blocks(current_state, distance_to_block(a.p + nmvect));
                                auto mid_state_b = merge_blocks(current_state, distance_to_block(b.p - nmvect));
                                mid_state_a['F'] = mid_state_b['F'] = std::max(next_state['F'], current_state['F']);
                                mid_state_a['G'] = mid_state_b['G'] = next_state['G'];
                                nsubprog.push_back(mid_state_a);
                                nsubprog.push_back(mid_state_b);
                                nsubprog.push_back(next_state);
                            } else {
                                //std::cout << "-- a.p " << a.p << "  transition_point.p " << transition_point.p << "   b.p " << b.p << std::endl;
                                auto mid_state = merge_blocks(current_state, distance_to_block(block_to_distance_t(current_state) + move_vec));
                                mid_state['G'] = next_state['G'];
                                mid_state['F'] = std::max(next_state['F'], current_state['F']);
                                nsubprog.push_back(mid_state);
                                nsubprog.push_back(next_state);
                            }
                        }
                        current_state = next_state;
                    } else {
                        if (block.at('G') == 92) {
                            current_state = merge_blocks(current_state, block);
                            nsubprog.push_back(current_state);
                        } else {
                            nsubprog.push_back(block);
                        }
                    }
                }
                nsubprog.shrink_to_fit();
                ret.push_back(nsubprog);
            } else {
                ret.push_back(subprogram);
            }
        }
    }
    return ret;
}


program_t remove_duplicate_blocks(const program_t& program_states, const block_t& initial_state)
{
    program_t ret;
    ret.reserve(program_states.size());
    auto current_state = merge_blocks({
                                          {'X', 0.0},
                                          {'Y', 0.0},
                                          {'Z', 0.0},
                                          {'A', 0.0},
                                          {'F', 0.1},
                                      },
        initial_state);
    for (auto s : program_states) {
        if (s.count('M') == 0) {
            if (s.count('G') &&
                ((s.at('G') == 0) || (s.at('G') == 1) || (s.at('G') == 92))) {
                auto new_state = merge_blocks(current_state, s);
                if (!((blocks_to_vector_move(new_state, current_state).length() == 0) &&
                        (new_state['F'] == current_state['F']))) {
                    auto nblock = diff_blocks(new_state, current_state);
                    nblock['G'] = new_state['G'];
                    ret.push_back(nblock);
                    //ret.push_back(new_state);
                    current_state = new_state;
                    //block_t diff_blocks(const block_t& destination, const block_t& source);

                } else {
                    current_state = new_state;
                }
                continue;
            }
        }
        ret.push_back(s);
    }
    ret.shrink_to_fit();
    return ret;
}


double get_feedrate_for_turn(double angle)
{
}


program_t apply_limits_for_turns(const program_t& program_states,
    const configuration::limits& machine_limits)
{
    auto ret_states = remove_duplicate_blocks(program_states, {});
    auto current_state = merge_blocks({}, {});
    for (auto& e : ret_states) {
        if (e.count('G')) {
            current_state = merge_blocks(current_state, e);
            e = current_state;
        }
    }
    //std::vector < std::pair < distance_t, double >
    if (ret_states.size() == 0) {
        return {};
    }
    if (ret_states.size() == 1) {
        auto& state = ret_states[0];
        if (state['F'] > 0) {
            state['F'] = std::min((machine_limits.max_no_accel_velocity_mm_s[0] +
                                      machine_limits.max_no_accel_velocity_mm_s[1] +
                                      machine_limits.max_no_accel_velocity_mm_s[2] +
                                      machine_limits.max_no_accel_velocity_mm_s[3]) /
                                      4,
                state['F']);
        }
        return ret_states;
    }
    {
        // first block, and last block
        auto first_diff = blocks_to_vector_move(ret_states[0], ret_states[1]);
        auto orig_state_f = ret_states[0]['F'];
        if (first_diff.length() > 0) {
            ret_states[0]['F'] = std::min(
                machine_limits.proportional_max_no_accel_velocity_mm_s(first_diff / first_diff.length()),
                orig_state_f);
        }
        if (std::isnan(ret_states[0]['F'])) {
            throw std::invalid_argument("A: ret_states[0]['F'] cannot be nan!!");
        }
        if (ret_states.size() == 2) {
            ret_states[1]['F'] = ret_states[0]['F'];
            return ret_states;
        }
    }
    //block_t previous_block;
    if (ret_states.size() > 2) {
        std::list<block_t> tristate;
        tristate.push_back(ret_states[0]);
        tristate.push_back(merge_blocks(tristate.back(), ret_states[1]));
        for (::size_t i = 1; i < ret_states.size() - 1; i++) {
            ret_states[i] = tristate.back();
            tristate.push_back(merge_blocks(tristate.back(), ret_states[i + 1]));
            // 0-90 - 0.25 of the min speed no accel to the 1.0 of the min speed no accel
            // 90-180 - linear scale from min speed no accel to max speed
            // merge_blocks
            auto A = block_to_distance_t(tristate.front());
            auto B = block_to_distance_t(*(++tristate.begin()));
            auto C = block_to_distance_t(tristate.back());

            // get minimum of the values for first vector and second vector
            double angle = B.angle(A, C);
            if (angle <= (M_PI / 2.0)) {
                auto y = linear_interpolation(angle, 0, 0.25, M_PI / 2.0, 1);
                if (ret_states[i]['F'] == 0.0) {
                    throw std::invalid_argument("feedrate cannot be 0:\n" + back_to_gcode({ret_states}));
                }
                double result_f = std::min(y * std::min(
                                                   machine_limits.proportional_max_no_accel_velocity_mm_s((B - A) / (B - A).length()),
                                                   machine_limits.proportional_max_no_accel_velocity_mm_s((C - B) / (C - B).length())),
                    ret_states[i]['F']);
                if (std::isnan(result_f)) throw std::invalid_argument("A: result_f cannot be nan!!");
                ret_states[i]['F'] = result_f;
            } else {
                auto B_A = (B - A).length();
                auto C_B = (C - B).length();
                B_A = (B_A <= 0) ? 0.0000001 : B_A;
                C_B = (C_B <= 0) ? 0.0000001 : C_B;
                auto angle_transformed = ((angle - M_PI / 2.0) / (M_PI / 2.0));
                angle_transformed *= angle_transformed;
                angle_transformed = angle_transformed * (M_PI / 2.0) + M_PI / 2.0;
                auto y = linear_interpolation(
                    angle_transformed,
                    M_PI / 2.0,
                    std::min(
                        machine_limits.proportional_max_no_accel_velocity_mm_s((B - A) / B_A),
                        machine_limits.proportional_max_no_accel_velocity_mm_s((C - B) / C_B)),
                    M_PI,
                    std::min(machine_limits.proportional_max_velocity_mm_s((B - A) / B_A),
                        machine_limits.proportional_max_velocity_mm_s((C - B) / C_B)));
                if (std::isnan(y)) y = ret_states[i]['F'];
                if (ret_states[i]['F'] == 0.0) {
                    throw std::invalid_argument("feedrate cannot be 0:\n" + back_to_gcode({ret_states}));
                }
                // std::cout << A << B << C << " || angle: " << angle << "y:" << y << " r_states['F']:" << (ret_states[i]['F']) << std::endl;
                double result_f =
                    std::min(y,
                        ret_states[i]['F']);
                if (std::isnan(result_f)) {
                    std::cout << y << std::endl;
                    std::cout << machine_limits.proportional_max_no_accel_velocity_mm_s((B - A) / B_A) << std::endl;
                    std::cout << machine_limits.proportional_max_no_accel_velocity_mm_s((C - B) / C_B) << std::endl;
                    std::cout << machine_limits.proportional_max_velocity_mm_s((B - A) / B_A) << std::endl;
                    std::cout << machine_limits.proportional_max_velocity_mm_s((C - B) / C_B) << std::endl;
                    throw std::invalid_argument("B: result_f cannot be nan!");
                }
                ret_states[i]['F'] = result_f;
            }
            tristate.pop_front();
        }
    }
    {
        auto second_diff = blocks_to_vector_move(*(--(--ret_states.end())), *(--ret_states.end()));
        auto A = machine_limits.proportional_max_no_accel_velocity_mm_s(second_diff / second_diff.length());
        auto B = (*(--ret_states.end()))['F'];
        auto ff = std::min(A, B);
        if (std::isnan(A)) ff = B;
        if (std::isnan(B)) ff = A;
        if (std::isnan(ff)) throw std::invalid_argument("ff feedrate cannot be nan");
        (*(--ret_states.end()))['F'] = ff;
    }
    return ret_states;
}


auto do_the_acceleration_limiting = [](auto program, const configuration::limits& machine_limits) {
    using namespace raspigcd::movement::physics;

    program_t result = program;
    //result.push_back(current_state);
    int walk_direction = 1;
    unsigned int prev_i = 0;

    {
        auto current_state = merge_blocks({
                                              {'X', 0.0},
                                              {'Y', 0.0},
                                              {'Z', 0.0},
                                              {'F', 0.1},
                                          },
            program.front());
        double prev_f = result.at(0).at('F');
        for (auto& e : result) {
            if (e.count('F')) {
                prev_f = e['F'];
            } else {
                e['F'] = prev_f;
            }
        }
    }
    bool fixing = true;
    while (fixing) {
        fixing = false;
        for (int i = 1; i < result.size(); i++) {
            auto A = block_to_distance_t(result[i - 1]);
            auto B = block_to_distance_t(result[i]);
            auto ABvec = B - A;
            double s = ABvec.length();
            if (s != 0) {
                if (result[i - 1]['F'] != result[i]['F']) {
                    double max_a = machine_limits.proportional_max_accelerations_mm_s2(ABvec / s);
                    double min_v = machine_limits.proportional_max_no_accel_velocity_mm_s(ABvec / s) / 2.0;
                    min_v = std::min(min_v, result[i]['F']);
                    max_a = std::max(max_a, min_v);
                    path_node_t pnA = {.p = A, .v = result[i - 1]['F']};
                    path_node_t pnB = {.p = B, .v = result[i]['F']};
                    double a_AB = acceleration_between(pnA, pnB);
                    if (std::abs(a_AB) > std::abs(max_a)) {
                        if (pnA.v > pnB.v) {
                            result[i - 1]['F'] = result[i - 1]['F'] * 0.8;
                            fixing = true;
                        } else if (pnA.v < pnB.v) {
                            result[i]['F'] = result[i]['F'] * 0.8;
                            fixing = true;
                        }
                    }
                }
            }
        }
    }
    return result;
};


program_t g1_move_to_g1_with_machine_limits(const program_t& program_states,
    const configuration::limits& machine_limits,
    block_t current_state0)
{
    using namespace raspigcd::movement::physics;
    if (program_states.size() == 0) throw std::invalid_argument("there must be at least one G0 or G1 code in the program!");
    program_t result;
    result.reserve(program_states.size() + 1024);
    block_t current_state = merge_blocks({{'X', 0.0}, {'Y', 0.0}, {'Z', 0.0}, {'A', 0.0}, {'F', 0.1}}, current_state0);
    result.push_back(current_state);
    for (const auto& ps_input : program_states) {
        auto next_state = merge_blocks(current_state, ps_input);
        if (ps_input.count('G')) {
            if ((ps_input.at('G') != 0) && (ps_input.at('G') != 1)) {
                throw std::invalid_argument("Gx should be the only type of the commands in the program for g1_move_to_g1_with_machine_limits");
            }
            //current_state
            auto A = block_to_distance_t(current_state);
            auto B = block_to_distance_t(next_state);
            auto ABvec = B - A;
            double s = ABvec.length();
            if (s == 0) {
                if (next_state.count('F')) result.back()['F'] = next_state['F'];
                //result.push_back(next_state);
            } else {
                result.push_back(next_state);
            }
        } else {
            throw std::invalid_argument("Gx should be the only type of the commands in the program for g1_move_to_g1_with_machine_limits");
        }
        current_state = next_state;
    }
    result.shrink_to_fit();
    auto result_with_limits = apply_limits_for_turns(result, machine_limits);
    if (result_with_limits.size() != result.size()) throw std::invalid_argument("result_with_limits shoud have equal size to result");

    // std::reverse(result_with_limits.begin(), result_with_limits.end());
    // result_with_limits = do_the_acceleration_limiting(result_with_limits, machine_limits);
    // std::reverse(result_with_limits.begin(), result_with_limits.end());
    result_with_limits = do_the_acceleration_limiting(result_with_limits, machine_limits);
    result_with_limits.erase(result_with_limits.begin());
    return result_with_limits;
}

program_t g0_move_to_g1_sequence(const program_t& program_states,
    const configuration::limits& machine_limits,
    block_t current_state)
{
    using namespace raspigcd::movement::physics;
    if (program_states.size() == 0) throw std::invalid_argument("there must be at least one G0 code in the program!");
    //double minimal_feedrate_from_gcode = 10000000.0;
    //for (const auto &e: program_states) {
    //    if (e.count('F')) minimal_feedrate_from_gcode = std::min(minimal_feedrate_from_gcode,e.at('F'));
    //}
    program_t result;
    for (const auto& ps_input : program_states) {
        auto next_state = merge_blocks(current_state, ps_input);
        if (ps_input.count('G')) {
            if (ps_input.at('G') != 0) throw std::invalid_argument("g0 should be the only type of the commands in the program for g0_move_to_g1_sequence");
            //current_state
            auto A = block_to_distance_t(current_state);
            auto B = block_to_distance_t(next_state);
            auto ABvec = B - A;
            double s = ABvec.length();
            if (s == 0) {
                result.push_back(next_state);
            } else {
                double a = machine_limits.proportional_max_accelerations_mm_s2(ABvec / s);
                double max_v = machine_limits.proportional_max_velocity_mm_s(ABvec / s);
                double min_v = machine_limits.proportional_max_no_accel_velocity_mm_s(ABvec / s);
                path_node_t pnA = {.p = A, .v = min_v};
                path_node_t pnMed = {.p = (A + B) * 0.5, .v = max_v};
                path_node_t pnB = {.p = B, .v = min_v};
                double a_real = acceleration_between(pnA, pnMed);
                //std::cout <<"a_real:" << a_real << " a_max" << a << std::endl;
                if (a_real >= a) {
                    auto block_A = current_state;
                    pnMed = calculate_transition_point(pnA, pnMed, a);
                    //std::cout << "pnMed.v " << pnMed.v << std::endl;
                    auto block_Med = merge_blocks(current_state, distance_to_block(pnMed.p));
                    auto block_B = next_state;
                    block_Med['F'] = pnMed.v;
                    block_Med['G'] = 1;
                    result.push_back(block_Med);
                    block_B['G'] = 1;
                    block_B['F'] = min_v;
                    result.push_back(block_B);
                } else {
                    auto block_A = current_state;
                    pnMed = calculate_transition_point(pnA, pnMed, a);
                    //std::cout << "pnMed.v " << pnMed.v << std::endl;
                    auto block_Med = merge_blocks(current_state, distance_to_block(pnMed.p));
                    block_Med['F'] = pnMed.v;
                    block_Med['G'] = 1;
                    result.push_back(block_Med);

                    pnMed = calculate_transition_point(pnB, pnMed, a);
                    block_Med = merge_blocks(current_state, distance_to_block(pnMed.p));
                    block_Med['F'] = pnMed.v;
                    block_Med['G'] = 1;
                    result.push_back(block_Med);

                    auto block_B = next_state;
                    block_B['G'] = 1;
                    block_B['F'] = min_v;
                    result.push_back(block_B);
                }
            }
        } else
            throw std::invalid_argument("g0 should be the only type of the commands in the program for g0_move_to_g1_sequence");
        current_state = next_state;
    }
    // for (const auto &e: result) {
    //     //if (e.count('F')) minimal_feedrate_from_gcode = std::min(minimal_feedrate_from_gcode,e.at('F'));
    //     if (e.count('F')) {
    //         if (e.at('F') < 0.001) {
    //             std::cerr << block_to_distance_with_v_t(e) << std::endl;
    //             throw std::invalid_argument("");
    //         }
    //     }
    // }
    return result;
}


std::string back_to_gcode(const partitioned_program_t& btg)
{
    std::stringstream strs;
    for (auto& group : btg) {
        strs << "; Group of size " << group.size() << std::endl;
        for (auto& block : group) {
            for (auto& e : block) {
                strs << "" << e.first << e.second << " ";
            }
            strs << std::endl;
        }
    }
    return strs.str();
}

block_t merge_blocks(const block_t& destination, const block_t& source)
{
    block_t merged = destination;
    for (const auto& sb : source) {
        merged[sb.first] = sb.second;
    }
    return merged;
}

// UNTESTED:
block_t diff_blocks(const block_t& destination, const block_t& source)
{
    block_t merged = destination;
    for (const auto& sb : source) {
        if (merged[sb.first] == sb.second) merged.erase(sb.first);
    }
    return merged;
}


partitioned_program_t group_gcode_commands(const program_t& program_states, const block_t& initial_state)
{
    partitioned_program_t generated_program;
    block_t current_state = merge_blocks({{'X', 0}, {'Y', 0}, {'Z', 0}, {'A', 0}}, initial_state);
    for (const auto& e : program_states) {
        if (generated_program.size() == 0) {
            if ((e.count('G') > 0) || (e.count('M')))
                generated_program.push_back({e});
            else
                throw std::invalid_argument("the first command must be G or M");
        } else {
            if (e.count('G')) {
                if (generated_program.back().back().count('G')) {
                    if ((generated_program.back().front().at('G') == e.at('G'))) {
                        generated_program.back().push_back(e);
                    } else {
                        generated_program.push_back({e});
                    }
                } else {
                    generated_program.push_back({e});
                }
            } else if (e.count('M')) {
                generated_program.push_back({e});
            } else {
                if (generated_program.back().front().count('G'))
                    generated_program.back().push_back(e);
                else
                    throw std::invalid_argument("It is not clear if the command is about G or M");
            }
        }
        current_state = merge_blocks(current_state, e);
        /// check correctness of first command
        if ((generated_program.back().size() == 1) && (generated_program.back()[0].count('G') == 1)) {
            if ((((int)generated_program.back()[0].at('G')) == 1) &&
                (((int)generated_program.back()[0].count('F')) == 0)) {
                //        // we must provide feedrate for the first command
                generated_program.back() = {{{'G', 1.0}, {'F', current_state.at('F')}}, e};
            }
        }
    }


    return generated_program;
}


program_t gcode_to_maps_of_arguments(const std::string& program_)
{
    // command_to_map_of_arguments
    program_t ret;
    std::regex re("[\r\n]");
    std::sregex_token_iterator
        first{program_.begin(), program_.end(), re, -1},
        last;
    int line_number = 0;
    for (auto line : std::vector<std::string>(first, last)) {
        try {
            auto cm = command_to_map_of_arguments(line);
            if (cm.size()) ret.push_back(cm);
        } catch (const std::invalid_argument& err) {
            throw std::invalid_argument(std::string("gcode_to_maps_of_arguments[") + std::to_string(line_number) + "]: \"" + line + "\" ::: " + err.what());
        }
        line_number++;
    }
    return ret;
}


std::map<char, double> command_to_map_of_arguments(const std::string& command__)
{
    //static std::regex command_rex("[^a-zA-Z0-9.\\-;]");
    static std::regex command_rex("[ \r\n\t]");
    for (auto c : command__)
        if (c == '\n') throw std::invalid_argument("new line is not allowed");
    std::map<char, std::string> ret_0;
    char cmndname = 0; // current command name
    std::string v = "";
    // write the results to an output iterator
    for (char c : std::regex_replace(command__, command_rex, "")) {
        if (c == ';') break;
        if ((c >= 'a') && (c <= 'z')) c = c + 'A' - 'a';
        if ((c >= 'A') && (c <= 'Z')) {
            cmndname = c;
            v = "";
            ret_0[cmndname] = v;
        } else {
            v = v + c;
            if (cmndname == 0) throw std::invalid_argument("gcode line cannot start with number");
            ret_0[cmndname] = v; //std::stod(v);
        }
    }
    std::map<char, double> ret;
    for (auto it = ret_0.begin(); it != ret_0.end();) {
        size_t _idx = 0;
        ret[it->first] = std::stod(it->second, &_idx);
        if (_idx < it->second.size()) throw std::invalid_argument("this is not a number");
        it = ret_0.erase(it);
    }
    return ret;
}


program_t optimize_path_douglas_peucker_g(const program_t& program_, const double epsilon, const block_t& p0_)
{
    std::vector<distance_with_velocity_t> path;
    path.reserve(program_.size());
    auto is_block_a_new_position = [](const block_t& e) {
        if ((e.count('M') == 0) && (e.count('G') != 0)) {
            switch ((int)(e.at('G'))) {
            case 0:
            case 1:
                return true;
            }
        }
        return false;
    };
    block_t machine_state = merge_blocks({{'X', 0}, {'Y', 0}, {'Z', 0}, {'F', 0.1}}, p0_);
    path.push_back(block_to_distance_with_v_t(machine_state));
    for (const auto& e : program_) {
        if (is_block_a_new_position(e)) {
            machine_state = merge_blocks(machine_state, e);
            auto p = block_to_distance_with_v_t(machine_state);
            path.push_back(p);
        }
    }

    std::vector<char> toDelete = optimize_generic_path_dp(epsilon, path);
    //std::cout << "PATH:" << std::endl;
    for (unsigned i = 0; i < path.size(); i++) {
        //auto e = path[i];
        //std::cout << e << " " << (toDelete[i]?"DELETE":"keep") << std::endl;
        if (i > 0) {
            if (path[i].back() != path[i - 1].back()) toDelete[i] = false;
            if (i > 1) {
            if (!is_block_a_new_position(program_[i-2])) toDelete[i] = false;
            }
        }
        if (i < path.size() - 1) {
            if (path[i].back() != path[i + 1].back()) toDelete[i] = false;
            if (i < path.size() - 2) {
                if (!is_block_a_new_position(program_[i])) toDelete[i] = false;
            }
        }
    }
    
    program_t ret;
    ret.reserve(program_.size());
    unsigned int idx_in_program = 0;
    auto qpsh = [&]() {
        while ((idx_in_program < program_.size()) && !is_block_a_new_position(program_.at(idx_in_program))) {
            //std::cout << "   i " << i << "  idx_in_program " << idx_in_program << std::endl;
            ret.push_back(program_.at(idx_in_program));
            idx_in_program++;
        }
    };
    for (unsigned int i = 1; i < path.size(); i++) {
        //        std::cout << "++ i " << i << "  idx_in_program " << idx_in_program << " ;;; toDelete: " <<
        //        (toDelete[i]?"delete":"keep") << std::endl;
        qpsh();
        if (!toDelete[i]) {
            //std::cout << "     push " << idx_in_program << std::endl;
            //path.push_back(tmp[i]);
            if (idx_in_program < program_.size())
                ret.push_back(program_.at(idx_in_program));
            else
                std::cout << "     push failed[" << i << "]:: " << idx_in_program << std::endl;
        }
        idx_in_program++;
    }
    qpsh();
    ret.shrink_to_fit();
    //for (auto &p:ret) if (p.count('F')) p['F'] /= 100.0;
    return ret;
}

program_t optimize_path_douglas_peucker(const program_t& program_, const double epsilon, const block_t& p0_)
{
    program_t ret;
    block_t state = p0_;
    for (auto e : group_gcode_commands(program_)) {
        // std::cout << "e: " << e.size() << std::endl;
        auto pp = optimize_path_douglas_peucker_g(e, epsilon, p0_);
        // std::cout << "pp: " << pp.size() << std::endl;
        state = last_state_after_program_execution(e, state);
        ret.insert(ret.end(), pp.begin(), pp.end());
    }
    return ret;
}
} // namespace gcd
} // namespace raspigcd
