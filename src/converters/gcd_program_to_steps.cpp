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


#include <converters/gcd_program_to_steps.hpp>
#include <movement/physics.hpp>
#include <movement/simple_steps.hpp>

#include <functional>

namespace raspigcd {
namespace converters {

//inline void smart_append(std::list<raspigcd::hardware::multistep_command>& fragment, const raspigcd::hardware::multistep_commands_t& steps_todo)
auto smart_append = [](auto& fragment, const raspigcd::hardware::multistep_commands_t& steps_todo) -> void 
{
/*    for (auto e : steps_todo) {
        if (e.count > 0) {
            if ((fragment.size() == 0) ||
                !(multistep_command_same_command(e, fragment.back()))) {
                fragment.push_back(e);
            } else {
                if (fragment.back().count > 0x0fffffff) {
                    fragment.push_back(e);
                } else {
                    fragment.back().count += e.count;
                }
            }
        }
    } */
    std::size_t i0 = 0;
    auto e = steps_todo[i0];
    if (e.count > 0) {
        if ((fragment.size() == 0) ||
            !(multistep_command_same_command(e, fragment.back()))) {
            fragment.push_back(e);
            i0++;
        }
    }
    for (std::size_t i = i0 ; i< steps_todo.size() ;i++) {
        e = steps_todo[i];
        if (e.count > 0) {
                if (fragment.back().count > 0x0fffffff) {
                    fragment.push_back(e);
                } else {
                    fragment.back().count += e.count;
                }
            
        }
    }
};


raspigcd::hardware::multistep_commands_t __generate_g1_steps(
    const raspigcd::gcd::block_t& state,
    const raspigcd::gcd::block_t& next_state,
    double dt,
    hardware::motor_layout& ml_)
{
    using namespace raspigcd::hardware;
    using namespace raspigcd::gcd;
    using namespace raspigcd::movement::simple_steps;
    using namespace movement::physics;
    auto pos_from = gcd::block_to_distance_t(state);
    auto pos_to = gcd::block_to_distance_t(next_state);

    double l = (pos_to - pos_from).length(); // distance to travel
    double v0 = state.at('F');               // velocity
    double v1 = next_state.at('F');          // velocity
    std::list<multistep_command> fragment;   // fraagment of the commands list generated in this stage
    steps_t final_steps;                     // steps after the move

    multistep_commands_t steps_todo;
    if (l > 0) {
        if ((v0 == v1)) {
            if (v1 == 0) throw std::invalid_argument("the feedrate should not be 0 for non zero distance");
            auto pos = pos_from;
            double s = v1 * dt; // distance to go
            auto direction = (pos_to - pos_from) / l;
            auto pos_from_steps = ml_.cartesian_to_steps(pos); //configuration(state);
            for (int i = 1; s <= l; ++i, s = v1 * (dt * i)) {
                // TODO: Create test case for this situation!!!!
                auto np = pos_from + direction * s;
                auto pos_to_steps = ml_.cartesian_to_steps(np); //gcd::block_to_distance_t(next_state);
                chase_steps(steps_todo, pos_from_steps, pos_to_steps);
                smart_append(fragment, steps_todo);
                steps_todo.clear();
                pos = np;
                pos_from_steps = pos_to_steps;
            }
            final_steps = pos_from_steps;
        } else if ((v1 != v0)) {
            auto direction = (pos_to - pos_from) / l;
            const path_node_t pn_a{.p = pos_from, .v = v0};
            const path_node_t pn_b{.p = pos_to, .v = v1};
            double a = acceleration_between(pn_a, pn_b);
            //std::cout << "a = " << a << std::endl;
            double t = dt;                                       ///< current time
            auto l = [&]() { return v0 * t + 0.5 * a * t * t; }; ///< current distance from p0
            double s = (pos_to - pos_from).length();             // distance to travel
            auto p_steps = ml_.cartesian_to_steps(pos_from);
            for (int i = 1; l() < s; ++i, t = dt * i) {
                auto pos = ml_.cartesian_to_steps(pos_from + direction * l());
                chase_steps(steps_todo, p_steps, pos);
                smart_append(fragment, steps_todo);
                steps_todo.clear();
                p_steps = pos;
            }
            final_steps = p_steps;
        }
        auto pos_to_steps = ml_.cartesian_to_steps(pos_to);
        if (!(final_steps == pos_to_steps)) { // fix missing steps
            chase_steps(steps_todo, final_steps, pos_to_steps);
            //fragment.insert(fragment.end(), steps_todo.begin(), steps_todo.end());
            smart_append(fragment, steps_todo);
            steps_todo.clear();
        }
        auto collapsed = collapse_repeated_steps(fragment);
        return collapsed;
    }
    return {};
}

hardware::multistep_commands_t program_to_steps(
    const gcd::program_t& prog_,
    const configuration::actuators_organization& conf_,
    hardware::motor_layout& ml_,
    const gcd::block_t initial_state_, // = {{'F',0}},
    std::function<void(const gcd::block_t)> finish_callback_f_)
{
    using namespace raspigcd::hardware;
    using namespace raspigcd::gcd;
    using namespace raspigcd::movement::simple_steps;
    using namespace movement::physics;
    auto state = initial_state_;
    std::list<multistep_command> result;
    //double dt = 0.000001 * (double)conf_.tick_duration_us;//
    double dt = ((double)conf_.tick_duration_us) / 1000000.0;
    //std::cout << "dt = " << dt << std::endl;
    for (const auto& block : prog_) {
        finish_callback_f_(state);
        auto next_state = gcd::merge_blocks(state, block);

        if (next_state.at('G') == 92) {
            // change position, but not generate steps
        } else if (next_state.at('G') == 4) {
            //std::cout << "G4: " << std::endl;
            double t = 0;
            if (block.count('X')) { // seconds
                t = block.at('X');
            } else if (block.count('P')) {
                t = block.at('P') / 1000.0;
            }
            hardware::multistep_command executor_command = {};
            executor_command.count = t / dt;
            result.push_back(executor_command);
            next_state = state;
        } else if ((next_state.at('G') == 1) || (next_state.at('G') == 0)) {
            auto collapsed = __generate_g1_steps(state, next_state, dt, ml_);
            result.insert(result.end(), collapsed.begin(), collapsed.end());
        }
        state = next_state;
    }
    finish_callback_f_(state);
    return collapse_repeated_steps(result);
}


hardware::multistep_commands_t bezier_spline_program_to_steps(
    const gcd::program_t& prog_,
    const configuration::actuators_organization& conf_,
    hardware::motor_layout& ml_,
    const gcd::block_t initial_state_, // = {{'F',0}},
    std::function<void(const gcd::block_t)> finish_callback_f_)
{
    double arc_length = 0.5;


    using namespace raspigcd::hardware;
    using namespace raspigcd::gcd;
    using namespace raspigcd::movement::simple_steps;
    using namespace movement::physics;

    // std::cout << back_to_gcode({prog_}) << std::endl;

    gcd::block_t state = initial_state_;
    std::list<multistep_command> result;
    double dt = ((double)conf_.tick_duration_us) / 1000000.0;
    std::vector<distance_with_velocity_t> distances;

    distances.push_back(block_to_distance_with_v_t(state));
    for (const auto& block : prog_) {
        finish_callback_f_(state);
        auto next_state = gcd::merge_blocks(state, block);

        if (next_state.at('G') == 92) {
            // change position, but not generate steps
            throw std::invalid_argument("G92 is not supported in spline mode");
        } else if (next_state.at('G') == 4) {
            throw std::invalid_argument("G4 is not supported in spline mode");
        } else if ((next_state.at('G') == 1) || (next_state.at('G') == 0)) {
            distances.push_back(block_to_distance_with_v_t(next_state));
        }
        state = next_state;
    }
    finish_callback_f_(state);
    distances = optimize_path_dp(distances, std::max(arc_length * 0.5, 0.01));
    // remove nodes that are touching each other (by using its average coordinate)
    distances = [&distances, &arc_length]() {
        std::vector<distance_with_velocity_t> ret;
        distance_with_velocity_t* prev = nullptr;
        for (auto& e : distances) {
            if (prev != nullptr) {
                distance_t a = {(*prev)[0], (*prev)[1], (*prev)[2]};
                distance_t b = {e[0], e[1], e[2]};
                if ((b - a).length() >= arc_length * e[3]) {
                    ret.push_back(e);
                } else {
                    ret.back() = e;
                }
            } else {
                ret.push_back(e);
            }
            prev = &e;
        }
        return ret;
    }();

    state = initial_state_;
    distance_with_velocity_t pp0 = distances.front();
    steps_t pos_from_steps = ml_.cartesian_to_steps({pp0[0], pp0[1], pp0[2], pp0[3]});
    std::list<multistep_command> fragment; // fraagment of the commands list generated in this stage
    for (auto& pp : distances) {
        pp.back() = std::max(pp.back(), 0.01); // make v more reasonable
    }

    beizer_spline<5>(distances, [&](const distance_with_velocity_t& position) {
        if (!(position == pp0)) {
            distance_t dest_pos = {position[0], position[1], position[2], position[3]};
            multistep_commands_t steps_todo;

            auto pos_to_steps = ml_.cartesian_to_steps(dest_pos);
            chase_steps(steps_todo, pos_from_steps, pos_to_steps);
            smart_append(result, steps_todo);
            //result.insert(result.end(), steps_todo.begin(), steps_todo.end());
            if (result.size() > 1024 * 1024 * 64) {
                std::cerr << "bezier_spline_program_to_steps: problem in generating steps - the size is too big: " << result.size() << "; p: " << position << std::endl;
                throw std::invalid_argument("bezier_spline_program_to_steps: problem in generating steps - the size is too big");
            }
            pos_from_steps = pos_to_steps;
        }
    },
        dt, arc_length);
    return collapse_repeated_steps(result);
}



hardware::multistep_commands_t linear_interpolation_to_steps(
    const gcd::program_t& prog_,
    const configuration::actuators_organization& conf_,
    hardware::motor_layout& ml_,
    const gcd::block_t initial_state_, // = {{'F',0}},
    std::function<void(const gcd::block_t)> finish_callback_f_)
{
    double arc_length = 0.5;


    using namespace raspigcd::hardware;
    using namespace raspigcd::gcd;
    using namespace raspigcd::movement::simple_steps;
    using namespace movement::physics;

    // std::cout << back_to_gcode({prog_}) << std::endl;

    gcd::block_t state = initial_state_;
    double dt = ((double)conf_.tick_duration_us) / 1000000.0;
    std::vector<distance_with_velocity_t> distances;
    distances.reserve(1000000);

    distances.push_back(block_to_distance_with_v_t(state));
    for (const auto& block : prog_) {
        finish_callback_f_(state);
        auto next_state = gcd::merge_blocks(state, block);

        if (next_state.at('G') == 92) {
            // change position, but not generate steps
            throw std::invalid_argument("G92 is not supported in spline mode");
        } else if (next_state.at('G') == 4) {
            throw std::invalid_argument("G4 is not supported in spline mode");
        } else if ((next_state.at('G') == 1) || (next_state.at('G') == 0)) {
            distances.push_back(block_to_distance_with_v_t(next_state));
        }
        state = next_state;
    }
    finish_callback_f_(state);
    distances.shrink_to_fit();
    //distances = optimize_path_dp(distances, std::max(arc_length * 0.5, 0.01));
    // remove nodes that are touching each other (by using its average coordinate)
    distances = [&distances, &arc_length]() {
        std::vector<distance_with_velocity_t> ret;
        ret.reserve(distances.size());
        distance_with_velocity_t* prev = nullptr;
        for (auto& e : distances) {
            if (prev != nullptr) {
                distance_t a = {(*prev)[0], (*prev)[1], (*prev)[2]};
                distance_t b = {e[0], e[1], e[2]};
                if ((b - a).length() >= arc_length * e[3]) {
                    ret.push_back(e);
                } else {
                    ret.back() = e;
                }
            } else {
                ret.push_back(e);
            }
            prev = &e;
        }
        ret.shrink_to_fit();
        return ret;
    }();

    state = initial_state_;
    distance_with_velocity_t pp0 = distances.front();
    steps_t pos_from_steps = ml_.cartesian_to_steps({pp0[0], pp0[1], pp0[2], pp0[3]});
    steps_t pos_to_steps = ml_.cartesian_to_steps({pp0[0], pp0[1], pp0[2], pp0[3]});
    std::list<multistep_command> fragment; // fraagment of the commands list generated in this stage
    for (auto& pp : distances) {
        pp.back() = std::max(pp.back(), 0.01); // make v more reasonable
    }
    std::vector<multistep_command> result;
    result.reserve(200000000);
    follow_path_with_velocity<5>(distances, [&](const distance_with_velocity_t& position) {
        //if (!(position == pp0)) {
            distance_t dest_pos = {position[0], position[1], position[2], position[3]};
            multistep_commands_t steps_todo;

            pos_to_steps = ml_.cartesian_to_steps(dest_pos);
            chase_steps(steps_todo, pos_from_steps, pos_to_steps);
            smart_append(result, steps_todo);
            //result.insert(result.end(), steps_todo.begin(), steps_todo.end());
            if (result.size() > 1024 * 1024 * 64) {
                std::cerr << "bezier_spline_program_to_steps: problem in generating steps - the size is too big: " << result.size() << "; p: " << position << std::endl;
                throw std::invalid_argument("bezier_spline_program_to_steps: problem in generating steps - the size is too big");
            }
            pos_from_steps = pos_to_steps;
        //}
    },
        dt, 0.025
);
    //return collapse_repeated_steps(result);
    result.shrink_to_fit();
    return result; //std::vector<multistep_command> (result.begin(), result.end());

}



program_to_steps_f_t program_to_steps_factory(const std::string f_name)
{
    if (f_name == "program_to_steps") {
        return program_to_steps;
    }
    if (f_name == "bezier_spline") {
        return bezier_spline_program_to_steps;
    }
    if (f_name == "linear_interpolation") {
        return linear_interpolation_to_steps;
    }
    throw std::invalid_argument("bad function name - available are program_to_steps bezier_spline linear_interpolation");
}


} // namespace converters
} // namespace raspigcd
