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


/*

This is simple program that uses the library. It will execute given GCode.

*/

#include <configuration.hpp>
#include <configuration_json.hpp>
#include <converters/gcd_program_to_steps.hpp>
#include <factories.hpp>
#include <gcd/remove_g92_from_gcode.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_busy_wait.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/driver/low_timers_wait_for.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping.hpp>
#include <hardware/thread_helper.hpp>
#include <m_help.hpp>
#include <queue_t.hpp>

#include <fstream>
#include <future>
#include <iostream>
#include <mutex>
#include <random>
#include <regex>
#include <sstream>
#include <streambuf>
#include <string>
#include <tuple>


using namespace raspigcd;
using namespace raspigcd::hardware;
using namespace raspigcd::gcd;


auto wait_for_component_to_start = [](auto m, int t = 3000) {
    if (m.count('P') == 1) {
        t = m.at('P');
    } else if (m.count('X') == 1) {
        t = 1000 * m.at('X');
    }
    if (t > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds((int)t));
    return t;
};

/*
gcode preprocessing
gcode execution:
 steps generator - producer - only one
 steps executor - consumer - only one

 machine class
*/


class steps_consumer_t
{
public:
    std::shared_ptr<fifo_c<multistep_command>> queue; // must be set to valid queue!!
    std::shared_ptr<low_timers> timers;
    std::shared_ptr<low_steppers> steppers;

    std::atomic<bool> cancel_execution;
    std::atomic<int> delay_microseconds;

    steps_consumer_t(
        std::shared_ptr<fifo_c<multistep_command>> queue_,
        std::shared_ptr<low_timers> timers_,
        std::shared_ptr<low_steppers> steppers_,
        int delay_microseconds_)
    {
        if (queue_.get() == nullptr) throw std::invalid_argument("queue must be valid pointer");
        queue = queue_;
        if (timers_.get() == nullptr) throw std::invalid_argument("timers must be valid pointer");
        timers = timers_;
        if (steppers_.get() == nullptr) throw std::invalid_argument("timers must be valid pointer");
        steppers = steppers_;
        delay_microseconds = delay_microseconds_;
        cancel_execution = false;

        {
            // start with non empty queue
            hardware::multistep_command executor_command = {};
            executor_command.count = 1;
            for (int i = 0; i < 100; i++)
                queue_->put(cancel_execution, executor_command);
        }
    }
    void run()
    {
        try {
            std::chrono::high_resolution_clock::time_point prev_timer = timers->start_timing();
            while (!cancel_execution) {
                auto s = queue->get(cancel_execution);
                if (s.flags.bits.program_finish_bit == 1) {
                    std::cout << "s.flags.bits.program_finish_bit: FINISH" << std::endl;
                    break;
                }
                while ((s.count > 0) && (!cancel_execution)) {
                    s.count--;
                    prev_timer = timers->wait_for_tick_us(prev_timer, delay_microseconds);
                    steppers->do_step(s.b);
                }
            }
        } catch (std::invalid_argument& e) {
            // good finish of the get method - we shoud finish execution
            std::cout << "finished execution of run() command on " << __FILE__ << ":" << __LINE__ << std::endl;
        }
        std::cout << "finished execution, cancel_execution = " << cancel_execution << std::endl;
    };
};

class cnc_executor_t
{
    execution_objects_t _machine;

    configuration::global _cfg;                        ///< configuration
    std::shared_ptr<fifo_c<multistep_command>> _queue; ///< commands queue - this is our buffer for steps
    std::atomic_bool _producer_cancel_execution;       ///< stop generating steps - this value will be used if we need to break the get_value_from_queue
    steps_consumer_t _steps_consumer;
    int _last_spindle_on_delay;
    std::map<int, double> _spindles_status;
    distance_with_velocity_t _current_position;

    std::mutex _execute_mutex;

    std::atomic<int> _break_execution;


    void chase_steps_exec(const steps_t start_pos_, const steps_t destination_pos_)
    {
        //hardware::multistep_commands_t ret;
        auto steps = start_pos_;
        hardware::multistep_command executor_command = {};
        executor_command.count = 1;
        int did_mod = 1;
        //ret.reserve(4096);
        do {
            did_mod = 0;
            for (unsigned int i = 0; i < steps.size(); i++) {
                if (destination_pos_[i] > steps[i]) {
                    steps[i]++;
                    executor_command.b[i].dir = 1;
                    executor_command.b[i].step = 1;
                    did_mod = 1;
                } else if (destination_pos_[i] < steps[i]) {
                    steps[i]--;
                    executor_command.b[i].dir = 0;
                    executor_command.b[i].step = 1;
                    did_mod = 1;
                } else {
                    executor_command.b[i].step = 0;
                }
            }
            //if (did_mod) {
            _queue->put(_producer_cancel_execution, executor_command);
            //}
        } while (did_mod); //while ((--stodo) > 0);
    };


    void chase_steps_exec_sync(const steps_t start_pos_, const steps_t destination_pos_)
    {
        //hardware::multistep_commands_t ret;
        auto steps = start_pos_;
        hardware::multistep_command executor_command = {};
        executor_command.count = 1;
        int did_mod = 1;
        //ret.reserve(4096);
        std::chrono::high_resolution_clock::time_point prev_timer = _machine.timer_drv->start_timing();
        do {
            did_mod = 0;
            for (unsigned int i = 0; i < steps.size(); i++) {
                if (destination_pos_[i] > steps[i]) {
                    steps[i]++;
                    executor_command.b[i].dir = 1;
                    executor_command.b[i].step = 1;
                    did_mod = 1;
                } else if (destination_pos_[i] < steps[i]) {
                    steps[i]--;
                    executor_command.b[i].dir = 0;
                    executor_command.b[i].step = 1;
                    did_mod = 1;
                } else {
                    executor_command.b[i].step = 0;
                }
            }
            //if (did_mod) {
            //_queue->put(_producer_cancel_execution, executor_command);
            auto s = executor_command;
            if (s.flags.bits.program_finish_bit == 1) {
                std::cout << "s.flags.bits.program_finish_bit: FINISH" << std::endl;
                break;
            }
            while ((s.count > 0)) {
                //_steps_counter += s.b[0].step + s.b[1].step + s.b[2].step;
                //_tick_index++;

                //machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, [](int k, int v) { std::cout << "ENDSTOP_X " << k << "  value=" << v << std::endl; });
                //machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, [](int k, int v) { std::cout << "ENDSTOP_Y " << k << "  value=" << v << std::endl; });
                //machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, [](int k, int v) { std::cout << "ENDSTOP_Z " << k << "  value=" << v << std::endl; });

                auto kstate = _machine.buttons_drv->keys_state();
                if ((kstate.at(low_buttons_default_meaning_t::ENDSTOP_X) > 0) ||
                    (kstate.at(low_buttons_default_meaning_t::ENDSTOP_Y) > 0) ||
                    (kstate.at(low_buttons_default_meaning_t::ENDSTOP_Z) > 0)) {
                    std::cerr << "ENDSTOP HIT!!!" << std::endl;
                    throw std::out_of_range("endstop hit");
                }

                prev_timer = _machine.timer_drv->wait_for_tick_us(prev_timer, _cfg.tick_duration_us);
                s.count--;
                _machine.steppers_drv->do_step(s.b);
            }

            //}
        } while (did_mod); //while ((--stodo) > 0);
    };

    /**
 * @brief this method executes movement. It does not take into account the fact, that it changes physical position of the machine head
 * 
 * @param distances list of nodes to visit - x,y,z,a,feedrate
 */
    void perform_moves_abs(const std::vector<distance_with_velocity_t> distances)
    {
        using namespace raspigcd::hardware;
        using namespace raspigcd::gcd;
        //using namespace raspigcd::movement::simple_steps;
        using namespace movement::physics;
        if (distances.size() < 1) return;
        double dt = ((double)_cfg.tick_duration_us) / 1000000.0;
        //distance_with_velocity_t from_dist, to_dist;
        steps_t pos_from_steps = _machine.motor_layout_->cartesian_to_steps(distances[0]); // ml_.cartesian_to_steps({pp0[0], pp0[1], pp0[2], pp0[3]});
        auto exec_path_part = [&](const distance_with_velocity_t& position) {
            if (_break_execution == 1) {
                _break_execution = 0;
                throw std::out_of_range("termination by _break_execution");
            }
            distance_t dest_pos = {position[0], position[1], position[2], position[3]};
            steps_t pos_to_steps = _machine.motor_layout_->cartesian_to_steps(dest_pos);
            chase_steps_exec(pos_from_steps, pos_to_steps);
            pos_from_steps = pos_to_steps;
        };
        follow_path_with_velocity<5>(
            distances, exec_path_part,
            dt, 0.025);
        exec_path_part(distances.back());
    };

    void perform_moves_abs_sync_check(const std::vector<distance_with_velocity_t> distances)
    {
        using namespace raspigcd::hardware;
        using namespace raspigcd::gcd;
        //using namespace raspigcd::movement::simple_steps;
        using namespace movement::physics;
        if (distances.size() < 1) return;
        double dt = ((double)_cfg.tick_duration_us) / 1000000.0;
        //distance_with_velocity_t from_dist, to_dist;
        steps_t pos_from_steps = _machine.motor_layout_->cartesian_to_steps(distances[0]); // ml_.cartesian_to_steps({pp0[0], pp0[1], pp0[2], pp0[3]});
        auto exec_path_part = [&](const distance_with_velocity_t& position) {
            if (_break_execution == 1) {
                _break_execution = 0;
                throw std::out_of_range("termination by _break_execution");
            }
            distance_t dest_pos = {position[0], position[1], position[2], position[3]};
            steps_t pos_to_steps = _machine.motor_layout_->cartesian_to_steps(dest_pos);
            chase_steps_exec_sync(pos_from_steps, pos_to_steps);
            pos_from_steps = pos_to_steps;
        };
        follow_path_with_velocity<5>(
            distances, exec_path_part,
            dt, 0.025);
        exec_path_part(distances.back());
    };


public:
    cnc_executor_t(const execution_objects_t machine_,
        //int tick_duration_us_,
        configuration::global cfg_,
        int buffer_size_for_moves_ = 30000) : _machine(machine_),
                                              _cfg(cfg_),
                                              _queue(std::make_shared<fifo_c<multistep_command>>(buffer_size_for_moves_)),
                                              _producer_cancel_execution(false),
                                              _steps_consumer(_queue, _machine.timer_drv, _machine.steppers_drv, _cfg.tick_duration_us),
                                              _break_execution(0)
    {
        _last_spindle_on_delay = 7000; // safe delay in miliseconds
        _current_position = {0.0, 0.0, 0.0, 0.0, 0.1};
        _spindles_status[0] = 0.0;
    }

    virtual ~cnc_executor_t()
    {
        _steps_consumer.cancel_execution = true;
    }

    /**
     * @brief executes only G0 moves
     * 
     * @param moves_buffer_ 
     */
    std::list<std::pair<std::size_t, std::string>> execute_g0_moves(
        const std::list<std::pair<std::size_t, std::string>>& moves_buffer_)
    {
        std::string error_code_name = "";
        auto iterator = moves_buffer_.begin();
        std::vector<distance_with_velocity_t> path_to_follow = {_current_position};
        for (; iterator != moves_buffer_.end(); iterator++) {
            auto line_parsed = command_to_map_of_arguments((*iterator).second);
            if ((line_parsed.count('G') == 0) || (line_parsed['G'] != 0)) break;
            auto pp0 = _current_position;
            auto pp1 = _current_position;

            _current_position = {
                line_parsed.count('X') ? line_parsed['X'] : _current_position[0],
                line_parsed.count('Y') ? line_parsed['Y'] : _current_position[1],
                line_parsed.count('Z') ? line_parsed['Z'] : _current_position[2],
                line_parsed.count('A') ? line_parsed['A'] : _current_position[3],
                line_parsed.count('F') ? line_parsed['F'] : _current_position[4]};

            auto pp2 = _current_position;
            auto pp3 = _current_position;

            if (((pp3 - pp0) * distance_with_velocity_t{1.0, 1.0, 1.0, 1.0, 0.0}).length() == 0) continue;

            double frac = 0.25;                                      // the portion of the segment that is dedicated to acceleration and breaking
            auto move_vec_norm = (pp3 - pp0) / (pp3 - pp0).length(); // the movement vector normalized to the length of 1
            double max_speed_frac = 1.0;                             // the fraction of maximal speed that will be achieved
            double max_speed_frac_d = 0.5;                           // the coefficient that is the negative powers of 2 for adjusting using binary search
            double max_acceleration = _cfg.proportional_max_accelerations_mm_s2(move_vec_norm);
            auto calculate_pp = [&]() {
                pp1 = pp0 * (1.0 - frac) + pp3 * (frac);
                pp2 = pp0 * (frac) + pp3 * (1.0 - frac);

                pp0.back() = _cfg.proportional_max_no_accel_velocity_mm_s(move_vec_norm);
                pp1.back() = max_speed_frac * _cfg.proportional_max_velocity_mm_s(move_vec_norm);
                pp2.back() = max_speed_frac * _cfg.proportional_max_velocity_mm_s(move_vec_norm);
                pp3.back() = _cfg.proportional_max_no_accel_velocity_mm_s(move_vec_norm);

                auto abaccel_vec = pp0 - pp1;
                double acceleration_l = distance_t(abaccel_vec).length();
                double t = (pp1.back() - pp0.back()) / max_acceleration;
                double s = pp0.back() * t + 0.5 * max_acceleration * t * t; // target distance acceleration
                return std::pair(s, acceleration_l);
            };

            // adjust frac
            for (int i = 0; i < 10; i++) {
                auto [s, acceleration_l] = calculate_pp();
                if (s < acceleration_l) {
                    frac -= 0.02;
                } else if (s > acceleration_l) {
                    frac += 0.02;
                }
                if (frac < 0.01) {
                    frac = 0.01;
                    break;
                }
                if (frac > 0.5) {
                    frac = 0.49;
                    break;
                }
            }
            // estimate best acceleration
            for (int i = 0; i < 10; i++) {
                auto [s, acceleration_l] = calculate_pp();
                if (s < acceleration_l) {
                    max_speed_frac += max_speed_frac_d;
                    max_speed_frac_d *= 0.5;
                    std::cerr << "max_speed_frac_d += ; => " << max_speed_frac << std::endl;
                } else if (s > acceleration_l) {
                    max_speed_frac -= max_speed_frac_d;
                    max_speed_frac_d *= 0.5;
                    std::cerr << "max_speed_frac_d -= ; => " << max_speed_frac << std::endl;
                } else {
                    std::cerr << "s === acceleration!!! " << s << " == " << acceleration_l << std::endl;
                    break;
                }
                if (max_speed_frac > 1.0001) {
                    max_speed_frac = 1.0;
                    break;
                }
            }

            path_to_follow.push_back(pp1);
            path_to_follow.push_back(pp2);
            path_to_follow.push_back(pp3);
        }
        try {
            perform_moves_abs(path_to_follow);
        } catch (const std::out_of_range& e) {
            error_code_name = e.what();
        }
        if (error_code_name != "") throw std::out_of_range(error_code_name);
        return {iterator, moves_buffer_.end()};
    }

    /**
     * @brief executes only G1 moves
     * 
     * @param moves_buffer_ 
     */
    std::list<std::pair<std::size_t, std::string>> execute_g1_moves(
        const std::list<std::pair<std::size_t, std::string>>& moves_buffer_)
    {
        std::string error_code_name = "";
        auto iterator = moves_buffer_.begin();
        _current_position = _machine.motor_layout_->steps_to_cartesian(_machine.steppers_drv->get_steps()); // synchronize steps
        _current_position[4] = *std::min_element(_cfg.max_no_accel_velocity_mm_s.begin(), _cfg.max_no_accel_velocity_mm_s.end());
        std::vector<distance_with_velocity_t> path_to_follow = {_current_position}; // always start with the first step
        // let's get every move we can perform
        while (iterator != moves_buffer_.end()) {
            auto line_parsed = command_to_map_of_arguments((*iterator).second);
            //std::cout << ">> " << (*iterator).first << ":" << (*iterator).second << std::endl;

            // if it is not G1, then we should stop
            if ((line_parsed.count('G') == 0) || (line_parsed['G'] != 1)) break;
            _current_position = {
                line_parsed.count('X') ? line_parsed['X'] : _current_position[0],
                line_parsed.count('Y') ? line_parsed['Y'] : _current_position[1],
                line_parsed.count('Z') ? line_parsed['Z'] : _current_position[2],
                line_parsed.count('A') ? line_parsed['A'] : _current_position[3],
                line_parsed.count('F') ? line_parsed['F'] : _current_position[4]};
            // we only add path element if it performs some move
            if (((path_to_follow.back() * distance_with_velocity_t{1, 1, 1, 1, 0}) - (_current_position * distance_with_velocity_t{1, 1, 1, 1, 0})).length() < 0.001) {
                path_to_follow.back()[4] = _current_position[4];
            } else {
                path_to_follow.push_back(_current_position);
            }
            iterator++;
        }
        // turn on the laser
        if (_cfg.spindles.at(0).mode == configuration::spindle_modes::LASER) {
            _machine.spindles_drv->spindle_pwm_power(0, _spindles_status[0]);
        }
        try {
            perform_moves_abs(path_to_follow);
        } catch (const std::out_of_range& e) {
            error_code_name = e.what();
        }
        // wait for empty queue - synchronize with the steppers
        while (_queue->size() > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // now we can turn off laser
        if (_cfg.spindles.at(0).mode == configuration::spindle_modes::LASER) {
            _machine.spindles_drv->spindle_pwm_power(0, 0);
        }
        // check errors
        if (error_code_name != "") throw std::out_of_range(error_code_name);
        return {iterator, moves_buffer_.end()};
    }

    std::list<std::pair<std::size_t, std::string>> execute_g28_moves(
        const std::list<std::pair<std::size_t, std::string>>& moves_buffer_)
    {
        std::string error_code_name = "";
        auto iterator = moves_buffer_.begin();
        std::vector<distance_with_velocity_t> path_to_follow = {_current_position};

        auto line_parsed = command_to_map_of_arguments((*iterator).second);

        auto pp0 = _current_position;
        auto pp1 = _current_position;

        _current_position = {
            line_parsed.count('X') ? line_parsed['X'] : _current_position[0],
            line_parsed.count('Y') ? line_parsed['Y'] : _current_position[1],
            line_parsed.count('Z') ? line_parsed['Z'] : _current_position[2],
            line_parsed.count('A') ? line_parsed['A'] : _current_position[3],
            line_parsed.count('F') ? line_parsed['F'] : _current_position[4]};

        auto pp2 = _current_position;
        auto pp3 = _current_position;

        if (((pp3 - pp0) * distance_with_velocity_t{1.0, 1.0, 1.0, 1.0, 0.0}).length() == 0) {
            iterator++;
            return {iterator, moves_buffer_.end()};
        }

        double frac = 0.25;                                      // the portion of the segment that is dedicated to acceleration and breaking
        auto move_vec_norm = (pp3 - pp0) / (pp3 - pp0).length(); // the movement vector normalized to the length of 1
        double max_speed_frac = 1.0;                             // the fraction of maximal speed that will be achieved
        double max_speed_frac_d = 0.5;                           // the coefficient that is the negative powers of 2 for adjusting using binary search
        double max_acceleration = _cfg.proportional_max_accelerations_mm_s2(move_vec_norm);
        auto calculate_pp = [&]() {
            pp1 = pp0 * (1.0 - frac) + pp3 * (frac);
            pp2 = pp0 * (frac) + pp3 * (1.0 - frac);

            pp0.back() = _cfg.proportional_max_no_accel_velocity_mm_s(move_vec_norm);
            pp1.back() = max_speed_frac * _cfg.proportional_max_velocity_mm_s(move_vec_norm);
            pp2.back() = max_speed_frac * _cfg.proportional_max_velocity_mm_s(move_vec_norm);
            pp3.back() = _cfg.proportional_max_no_accel_velocity_mm_s(move_vec_norm);

            auto abaccel_vec = pp0 - pp1;
            double acceleration_l = distance_t(abaccel_vec).length();
            double t = (pp1.back() - pp0.back()) / max_acceleration;
            double s = pp0.back() * t + 0.5 * max_acceleration * t * t; // target distance acceleration
            return std::pair(s, acceleration_l);
        };

        // adjust frac
        for (int i = 0; i < 10; i++) {
            auto [s, acceleration_l] = calculate_pp();
            if (s < acceleration_l) {
                frac -= 0.02;
            } else if (s > acceleration_l) {
                frac += 0.02;
            }
            if (frac < 0.01) {
                frac = 0.01;
                break;
            }
            if (frac > 0.5) {
                frac = 0.49;
                break;
            }
        }
        // estimate best acceleration
        for (int i = 0; i < 10; i++) {
            auto [s, acceleration_l] = calculate_pp();
            if (s < acceleration_l) {
                max_speed_frac += max_speed_frac_d;
                max_speed_frac_d *= 0.5;
                std::cerr << "max_speed_frac_d += ; => " << max_speed_frac << std::endl;
            } else if (s > acceleration_l) {
                max_speed_frac -= max_speed_frac_d;
                max_speed_frac_d *= 0.5;
                std::cerr << "max_speed_frac_d -= ; => " << max_speed_frac << std::endl;
            } else {
                std::cerr << "s === acceleration!!! " << s << " == " << acceleration_l << std::endl;
                break;
            }
            if (max_speed_frac > 1.0001) {
                max_speed_frac = 1.0;
                break;
            }
        }
        path_to_follow.push_back(pp1);
        path_to_follow.push_back(pp2);
        path_to_follow.push_back(pp3);
        //iterator++;
        try {
            perform_moves_abs_sync_check(path_to_follow);
        } catch (const std::out_of_range& e) {
            error_code_name = e.what();
        }
        if (error_code_name != "") throw std::out_of_range(error_code_name);
        iterator++;
        return {iterator, moves_buffer_.end()};
    }

    distance_t get_position() const
    {
        return _current_position;
    }

    distance_with_velocity_t get_position_with_feedrate() const
    {
        return _current_position;
    }

    /**
     * @brief executes gcode string
     * 
     * @param program_ 
     */
    void execute_gcode(std::string program_)
    {
        if (_execute_mutex.try_lock()) {
            // command_to_map_of_arguments
            std::regex re("[\r\n]");
            std::sregex_token_iterator
                first{program_.begin(), program_.end(), re, -1},
                last;
            std::vector<std::string> moves_buffer;
            auto lines = std::vector<std::string>(first, last);
            std::list<std::pair<std::size_t, std::string>> lines_w_numbers;
            {
                size_t n = 0;
                std::transform(lines.begin(), lines.end(), std::back_inserter(lines_w_numbers),
                    [&n](const std::string s) -> std::pair<std::size_t, std::string> { return {n++, s}; });
            }
            while (lines_w_numbers.size() > 0) {
                std::cout << lines_w_numbers.front().first << ":" << lines_w_numbers.front().second << std::endl;
                try {
                    auto m = command_to_map_of_arguments(lines_w_numbers.front().second);
                    if (m.count('G')) {
                        std::string error_code_name = "";
                        _steps_consumer.cancel_execution = false;
                        std::thread _consumer_thread = std::thread([this]() {
                            set_thread_realtime();
                            _steps_consumer.run();
                        });
                        std::cout << "[I] running moves" << std::endl;
                        if (m['G'] == 0) {
                            try {
                                lines_w_numbers = execute_g0_moves(lines_w_numbers);
                            } catch (const std::out_of_range& e) {
                                error_code_name = e.what();
                            }
                        } else if (m['G'] == 1) {
                            try {
                                lines_w_numbers = execute_g1_moves(lines_w_numbers);
                            } catch (const std::out_of_range& e) {
                                error_code_name = e.what();
                            }
                        } else if ((int)m['G'] == 92) {
                            _current_position = {
                                m.count('X') ? m['X'] : _current_position[0],
                                m.count('Y') ? m['Y'] : _current_position[1],
                                m.count('Z') ? m['Z'] : _current_position[2],
                                m.count('A') ? m['A'] : _current_position[3],
                                m.count('F') ? m['F'] : _current_position[4]};
                            lines_w_numbers.pop_front();
                            auto steps = _machine.motor_layout_->cartesian_to_steps(_current_position);
                            _machine.steppers_drv->set_steps(steps);
                        } else if (m['G'] == 28) {
                            try {
                                lines_w_numbers = execute_g28_moves(lines_w_numbers);
                            } catch (const std::out_of_range& e) {
                                //error_code_name = e.what();
                                std::cerr << "Broken on: " << e.what() << std::endl;
                            }
                        } else {
                            lines_w_numbers.pop_front();
                        }
                        {
                            while (_queue->size() > 0)
                                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                            auto p = _machine.motor_layout_->steps_to_cartesian(_machine.steppers_drv->get_steps());
                            _current_position[0] = p[0];
                            _current_position[1] = p[1];
                            _current_position[2] = p[2];
                            _current_position[3] = p[3];
                        }
                        multistep_command terminate_command;
                        terminate_command.flags.bits.program_finish_bit = 1;
                        _queue->put(_steps_consumer.cancel_execution, terminate_command);
                        _consumer_thread.join(); // and sync
                        if (error_code_name == "") {
                            std::cout << "[I] finished G command - OK" << std::endl;
                        } else {
                            std::cout << "[I] finished G command - error: " << error_code_name << std::endl;
                            auto p_real = _machine.motor_layout_->steps_to_cartesian(_machine.steppers_drv->get_steps()); // ml_.cartesian_to_steps({pp0[0], pp0[1], pp0[2], pp0[3]});
                            for (int i = 0; i < 4; i++)
                                _current_position[i] = p_real[i];
                            _current_position[4] = 0.1;
                            _execute_mutex.unlock();
                            throw std::out_of_range(error_code_name);
                        }
                    } else if (m.count('M')) {
                        switch ((int)(m.at('M'))) {
                        case 17:
                            _machine.steppers_drv->enable_steppers({true, true, true, true});
                            wait_for_component_to_start(m, 200);
                            break;
                        case 18:
                            _machine.steppers_drv->enable_steppers({false, false, false, false});
                            wait_for_component_to_start(m, 200);
                            break;
                        case 3:
                            _spindles_status[0] = 1.0;
                            if (_cfg.spindles.at(0).mode != configuration::spindle_modes::LASER) {
                                _machine.spindles_drv->spindle_pwm_power(0, _spindles_status[0]);
                            }
                            _last_spindle_on_delay = wait_for_component_to_start(m, 3000);
                            break;
                        case 5:
                            _spindles_status[0] = 0.0;
                            if (_cfg.spindles.at(0).mode != configuration::spindle_modes::LASER) {
                                _machine.spindles_drv->spindle_pwm_power(0, _spindles_status[0]);
                            }
                            wait_for_component_to_start(m, 3000);
                            break;
                        }

                        lines_w_numbers.pop_front();
                    } else {
                        std::cout << "[I] skipping " << lines_w_numbers.front().first << ":" << lines_w_numbers.front().second << std::endl;
                        lines_w_numbers.pop_front();
                    }
                } catch (const std::invalid_argument& err) {
                    _execute_mutex.unlock();
                    auto line_number = lines_w_numbers.front().first;
                    throw std::invalid_argument(std::string("gcode_to_maps_of_arguments on line ") +
                                                std::to_string(line_number) + ": \"" + lines[line_number] + "\" ::: " + err.what());
                }
            }
            _execute_mutex.unlock();
        } else {
            throw std::runtime_error("only one command can be executed at a time");
        }
    }

    void stop()
    {
        using namespace std::chrono_literals;
        _break_execution = 1;
        int prevdelay = _steps_consumer.delay_microseconds;
        for (int i = 0; i < 1000; i++) {
            _steps_consumer.delay_microseconds = prevdelay + i;
        }
        while (_break_execution == 1) {
            std::this_thread::sleep_for(10us);
        }
        _steps_consumer.delay_microseconds = prevdelay;
    }

    /**
 * serialize state
 * */
    nlohmann::json get_state()
    {
        nlohmann::json ret;
        ret["last_spindle_on_delay"] = _last_spindle_on_delay;
        for (auto [k, v] : _spindles_status)
            ret["spindles_status"][k] = v;
        ret["current_position"] = _current_position;
        if (_execute_mutex.try_lock()) {
            _execute_mutex.unlock();
            ret["execute_gcode"] = "idle";
        } else {
            ret["execute_gcode"] = "running";
        }
        ret["steps"] = _machine.steppers_drv->get_steps();
        return ret;
    }
    /**
     * load serialized state
     * */
    void set_state(const nlohmann::json state_serialized)
    {
        std::cout << state_serialized.dump() << std::endl;
        _last_spindle_on_delay = state_serialized["last_spindle_on_delay"];
        int ii = 0;
        for (auto i : state_serialized.at("spindles_status"))
            _spindles_status[ii++] = i;
        for (int i = 0; i < _current_position.size(); i++)
            _current_position[i] = state_serialized["current_position"][i];
    }
};

auto interactive_go = [](auto& cfg, auto& execution_thread, auto gcode_program, auto& executor) {
    if (execution_thread.joinable()) {
        execution_thread.join();
    }
    execution_thread = std::thread([&, gcode_program]() {
        try {
            std::cout << "EXECUTE_START: " << executor.get_position() << std::endl;
            executor.execute_gcode(gcode_program);
            std::cout << "EXECUTE_DONE: " << executor.get_position() << std::endl;
        } catch (const std::runtime_error& e) {
            std::cerr << "EXECUTION_FAILED: you cannot run multiple programs at the same time. err: " << e.what() << std::endl;
        } catch (const std::invalid_argument& e) {
            std::cerr << "EXECUTION_FAILED: " << e.what() << std::endl;
        } catch (const std::out_of_range& e) {
            std::string errors = e.what();
            while (errors != "") {
                try {
                    executor.execute_gcode("M5P100\nG0Z5\nG0X0Y0\nG0X0Y0Z0");
                    errors = "";
                } catch (const std::out_of_range& eign) {
                    errors = eign.what();
                }
            }
            std::cout << "STOPPED: " << executor.get_position() << std::endl;
        }
    });
};


double fake_execution_and_statistics_collect(configuration::global cfg, std::function<void(execution_objects_t& machine)> work_on_machine_f)
{
    double execution_seconds = 0;

    auto steppers_drv = std::make_shared<driver::inmem>();
    auto spindles_drv = std::make_shared<raspigcd::hardware::driver::low_spindles_pwm_fake>(
        [](const int /*s_i*/, const double /*p_i*/) {

        });
    auto buttons_drv = std::make_shared<driver::low_buttons_fake>(10);

    std::shared_ptr<motor_layout> motor_layout_ = motor_layout::get_instance(cfg);
    motor_layout_->set_configuration(cfg);
    auto timer_drv = std::make_shared<hardware::driver::low_timers_fake>();
    timer_drv->on_wait_s = [&execution_seconds](double dt) {
        execution_seconds += (double)0.000001 * (double)dt;
    };
    std::shared_ptr<stepping_simple_timer> stepping = std::make_shared<stepping_simple_timer>(cfg, steppers_drv, timer_drv);

    execution_objects_t machine = {
        timer_drv,
        steppers_drv,
        spindles_drv,
        buttons_drv,
        motor_layout_,
        stepping};

    timer_drv->start_timing();
    work_on_machine_f(machine);

    return execution_seconds;
}

auto interactive_sim_go = [](auto& cfg, auto& execution_thread, auto gcode_program, auto& executor) {
    if (execution_thread.joinable()) {
        execution_thread.join();
    }
    execution_thread = std::thread([&, gcode_program]() {
        try {
            double executio_time = fake_execution_and_statistics_collect(cfg, [&](execution_objects_t& machine) {
                cnc_executor_t executor_fake(machine, cfg);
                executor_fake.set_state(executor.get_state());
                executor_fake.execute_gcode(gcode_program);
            });
            std::cout << "SIM_GO_TIME: " << executio_time << std::endl;
        } catch (const std::out_of_range& e) {
            std::cout << "STOPPED: " << executor.get_position() << std::endl;
        } catch (std::exception& e) {
            std::cout << "SIM_GO_TIME: "
                      << "ERROR: " << e.what() << std::endl;
        }
    });
};

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;
    std::vector<std::string> args(argv, argv + argc);
    configuration::global cfg;
    cfg.load_defaults();

    bool raw_gcode = false; // should I push G commands directly, without adaptation to machine
    for (unsigned i = 1; i < args.size(); i++) {
        if ((args.at(i) == "-h") || (args.at(i) == "--help")) {
            help_text(args);
        } else if (args.at(i) == "-c") {
            i++;
            cfg.load(args.at(i));
        } else if (args.at(i) == "-C") {
            std::cout << cfg << std::endl;
        } else if (args.at(i) == "--raw") {
            raw_gcode = true;
        } else if (args.at(i) == "--configtest") { // from stdin
            i++;


            auto machine = stepping_simple_timer_factory(cfg);
            machine.buttons_drv->on_key(low_buttons_default_meaning_t::PAUSE, [](int k, int v) { std::cout << "PAUSE     " << k << "  value=" << v << std::endl; });
            machine.buttons_drv->on_key(low_buttons_default_meaning_t::TERMINATE, [](int k, int v) { std::cout << "TERMINATE " << k << "  value=" << v << std::endl; });
            machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, [](int k, int v) { std::cout << "ENDSTOP_X " << k << "  value=" << v << std::endl; });
            machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, [](int k, int v) { std::cout << "ENDSTOP_Y " << k << "  value=" << v << std::endl; });
            machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, [](int k, int v) { std::cout << "ENDSTOP_Z " << k << "  value=" << v << std::endl; });
            int buffer_size_for_moves = 3000;
            auto queue = std::make_shared<fifo_c<multistep_command>>();
            cnc_executor_t executor(machine, cfg, buffer_size_for_moves);

            std::thread execution_thread;
            while (!std::cin.eof()) {
                std::string cmnd;
                std::cin >> cmnd;
                std::cout << "II: command: " << cmnd << std::endl;
                ;
                if (cmnd == "stop") {
                    executor.stop();
                } else if (cmnd == "status") {
                    auto status = executor.get_state();
                    raspigcd::steps_t end_steps = {
                        status["steps"][0],
                        status["steps"][1],
                        status["steps"][2],
                        status["steps"][3],
                    }; //machine.steppers_drv->get_steps();
                    raspigcd::steps_t end_pos = {
                        status["current_position"][0],
                        status["current_position"][1],
                        status["current_position"][2],
                        status["current_position"][3],
                    }; //machine.steppers_drv->get_steps();
                    std::cout << "STEPS: ";
                    for (auto v : end_steps)
                        std::cout << v << " ";
                    if (status["execute_gcode"] == "running") {
                        std::cout << "STATUS: " << end_pos << " running" << std::endl;
                    } else {
                        std::cout << "STATUS_ELEMENT: ";
                        //for (auto [k, v] : machine_status_after_exec)
                        //    std::cout << " " << k << "=" << v;
                        std::cout << std::endl;
                        std::cout << "STATUS: " << end_pos << " idle" << std::endl;
                    }
                    /*

                    std::cout << "STATUS: " << status["current_position"] << " " << status["execute_gcode"] << std::endl; */
                } else if ((cmnd == "execute") || (cmnd == "exec")) {
                    std::string filename;
                    std::getline(std::cin, filename);
                    filename = std::regex_replace(filename, std::regex("^ +"), "");
                    using namespace raspigcd;
                    using namespace raspigcd::hardware;

                    std::ifstream gcd_file(filename);
                    if (!gcd_file.is_open()) throw std::invalid_argument("could not open file \"" + filename + "\"");
                    std::string gcode_program((std::istreambuf_iterator<char>(gcd_file)),
                        std::istreambuf_iterator<char>());

                    std::cout << "EXECUTE: \"" << filename << "\"" << std::endl;
                    interactive_go(cfg, execution_thread, gcode_program, executor);
                    std::cout << "EXECUTE_FINISHED: \"" << filename << "\"" << std::endl;
                    std::cout << "EXECUTE_DONE: " << executor.get_state()["current_position"] << std::endl;
                    /*
                    if (execute_promise.valid()) {
                        std::cout << "task is currently executing..." << std::endl;
                    } else {
                        cancel_execution = false;
                        execute_promise = std::async([&, filename]() {
                            low_buttons_handlers_guard low_buttons_handlers_guard_(machine.buttons_drv);
                            std::cout << "EXECUTE: \"" << filename << "\"" << std::endl;
                            auto [err_code, machine_state] = execute_gcode_file(cfg, raw_gcode, filename, machine, cancel_execution, machine_status_after_exec);
                            std::cout << "EXECUTE_FINISHED: \"" << filename << "\"" << std::endl;
                            auto end_pos = machine.motor_layout_->steps_to_cartesian(machine.steppers_drv->get_steps());
                            machine_state['X'] = end_pos[0];
                            machine_state['Y'] = end_pos[1];
                            machine_state['Z'] = end_pos[2];
                            std::cout << "EXECUTE_DONE: " << end_pos << std::endl;
                            return machine_state;
                        }); */
                } else if (cmnd == "go") {
                    std::string gcode_program = "; direct gcode interpretation start";
                    bool multiline = false;
                    std::string l;
                    while (std::getline(std::cin, l)) {
                        if (multiline && (l.find(";multiline") != std::string::npos)) break;
                        static const std::regex r("[\\\\][n]");
                        l = std::regex_replace(l, r, "\n");
                        gcode_program += "\n" + l;

                        if (!multiline) {
                            if (l.find(";multiline") != std::string::npos)
                                multiline = true;
                            else
                                break;
                        }
                    }
                    interactive_go(cfg, execution_thread, gcode_program, executor);
                } else if (cmnd == "sim_go") {
                    std::string gcode_program = "; direct gcode interpretation start";
                    bool multiline = false;
                    std::string l;
                    while (std::getline(std::cin, l)) {
                        if (multiline && (l.find(";multiline") != std::string::npos)) break;
                        static const std::regex r("[\\\\][n]");
                        l = std::regex_replace(l, r, "\n");
                        gcode_program += "\n" + l;

                        if (!multiline) {
                            if (l.find(";multiline") != std::string::npos)
                                multiline = true;
                            else
                                break;
                        }
                    }
                    interactive_sim_go(cfg, execution_thread, gcode_program, executor);
                } else if (cmnd == "q") {
                    break;
                } else if (cmnd == "sync") {
                    if (execution_thread.joinable()) {
                        execution_thread.join();
                    }
                } else {
                    std::cerr << "EE: Wrong command: " << cmnd << std::endl;
                }
            }
        }
    }

    return 0;
}
