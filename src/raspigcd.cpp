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

#include <configuration_json.hpp>

#include <fstream>
#include <future>
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

void help_text(const std::vector<std::string>& args)
{
    std::cout << "NAME" << std::endl;
    std::cout << "\t" << args[0] << " - raspigcd runner program." << std::endl;
    std::cout << std::endl;
    std::cout << "SYNOPSIS" << std::endl;
    std::cout << "\t" << args[0] << " [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "DESCRIPTION" << std::endl;
    std::cout << "\tIt allows for execution of gcode on Raspberry Pi and simulation of such on desktop." << std::endl;
    std::cout << std::endl;
    std::cout << "\t-c <configfile>" << std::endl;
    std::cout << "\t\tprovide configuration file JSON" << std::endl;
    std::cout << std::endl;
    std::cout << "\t-C" << std::endl;
    std::cout << "\t\tdisplay current configuration in JSON format" << std::endl;
    std::cout << std::endl;
    std::cout << "\t-f <filename>" << std::endl;
    std::cout << "\t\tgcode file to execute" << std::endl;
    std::cout << std::endl;
    std::cout << "\t-h" << std::endl;
    std::cout << "\t\thelp screen" << std::endl;
    std::cout << std::endl;
    std::cout << "\t--raw" << std::endl;
    std::cout << "\t\tTreat the file as raw - no additional processing. No machine limits check (speed, acceleration, ...)." << std::endl;
    std::cout << std::endl;
    std::cout << "\t--configtest" << std::endl;
    std::cout << "\t\tEnables the debug mode for testing configuration" << std::endl;
    std::cout << std::endl;
    std::cout << "AUTHOR" << std::endl;
    std::cout << "\tTadeusz Puźniakowski" << std::endl;
    std::cout << std::endl;
    std::cout << "REPORTING BUGS" << std::endl;
    std::cout << "\tOnline at <https://github.com/pantadeusz/raspigcd2>" << std::endl;
    std::cout << std::endl;
    std::cout << "COPYRIGHT" << std::endl;
    std::cout << "\t" << std::endl;
    std::cout << "\tRaspberry Pi G-CODE interpreter" << std::endl;
    std::cout << "\t" << std::endl;
    std::cout << "\tCopyright (C) 2019 Tadeusz Puźniakowski puzniakowski.pl" << std::endl;
    std::cout << "\t" << std::endl;
    std::cout << "\tThis program is free software: you can redistribute it and/or modify" << std::endl;
    std::cout << "\tit under the terms of the GNU Affero General Public License as published by" << std::endl;
    std::cout << "\tthe Free Software Foundation, either version 3 of the License, or" << std::endl;
    std::cout << "\t(at your option) any later version." << std::endl;
    std::cout << "\t" << std::endl;
    std::cout << "\tThis program is distributed in the hope that it will be useful," << std::endl;
    std::cout << "\tbut WITHOUT ANY WARRANTY; without even the implied warranty of" << std::endl;
    std::cout << "\tMERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the" << std::endl;
    std::cout << "\tGNU Affero General Public License for more details." << std::endl;
    std::cout << "\t" << std::endl;
    std::cout << "\tYou should have received a copy of the GNU Affero General Public License" << std::endl;
    std::cout << "\talong with this program.  If not, see <https://www.gnu.org/licenses/>." << std::endl;
}


partitioned_program_t preprocess_program_parts(partitioned_program_t program_parts, const configuration::global& cfg, block_t machine_state)
{
    program_t prepared_program;

    for (auto& ppart : program_parts) {
        if (ppart.size() != 0) {
            if (ppart[0].count('M') == 0) {
                //std::cout << "G PART: " << ppart.size() << std::endl;
                switch ((int)(ppart[0]['G'])) {
                case 0:
                    ppart = g1_move_to_g1_with_machine_limits(ppart, cfg, machine_state);
                    prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                    machine_state = last_state_after_program_execution(ppart, machine_state);
                    break;
                case 1:
                    ppart = g1_move_to_g1_with_machine_limits(ppart, cfg, machine_state, false);
                    prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                    machine_state = last_state_after_program_execution(ppart, machine_state);
                    break;
                case 4:
                case 28:
                case 92:
                    prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                    machine_state = merge_blocks(machine_state, ppart.front());
                    break;
                }
            } else {
                //std::cout << "M PART: " << ppart.size() << std::endl;
                for (auto& m : ppart) {
                    switch ((int)(m['M'])) {
                    case 18:
                    case 3:
                    case 5:
                    case 17:
                        prepared_program.push_back(m);
                        break;
                    }
                }
            }
        }
    }

    prepared_program = optimize_path_douglas_peucker(prepared_program, cfg.douglas_peucker_marigin);
    program_parts = group_gcode_commands(remove_duplicate_blocks(prepared_program, {}));
    return program_parts;
}


void execute_calculated_multistep(raspigcd::hardware::multistep_commands_t m_commands, execution_objects_t machine, std::function<void(int, int)> on_stop_execution, std::atomic<bool>& cancel_execution, std::atomic<bool>& paused, long int last_spindle_on_delay, std::map<int, double>& spindles_status)
{
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, on_stop_execution);
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, on_stop_execution);
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, on_stop_execution);

    machine.stepping->exec(m_commands, [machine, &cancel_execution, &paused, last_spindle_on_delay, &spindles_status](auto, auto tick_n) -> int {
        std::cout << "break at " << tick_n << " tick" << std::endl;
        for (auto e : spindles_status) {
            // stop spindles and lasers ASAP!
            machine.spindles_drv->spindle_pwm_power(e.first, 0);
        }
        if (cancel_execution) return 0; // stop execution
        while ((paused) && (!cancel_execution)) {
            machine.timer_drv->wait_us(10000);
        }
        if (cancel_execution) return 0; // stop execution

        for (auto e : spindles_status) {
            machine.spindles_drv->spindle_pwm_power(e.first, e.second);
            machine.timer_drv->wait_us(1000 * last_spindle_on_delay);
        }
        return 1; // continue execuiton
    });
}

void home_position_find(char axis_id, 
    double direction_value,
    converters::program_to_steps_f_t program_to_steps, 
    configuration::global cfg, 
    execution_objects_t machine, 
    std::atomic<bool>& is_cancel_execution, 
    std::atomic<bool>& is_paused, 
    long int last_spindle_on_delay, 
    std::map<int, double>& spindles_status,
    const double goto_default_feedrate = 50.0)
{
    std::function<void(int, int)> on_stop_execution = [](int, int) {};

    std::map<char, std::tuple<double, double, double>> directions = {
        {'X', {(direction_value==0.0)?2000:direction_value, std::min(5.0, cfg.max_no_accel_velocity_mm_s[0]), std::min(goto_default_feedrate, cfg.max_velocity_mm_s[0])}},
        {'Y', {(direction_value==0.0)?4000:direction_value, std::min(5.0, cfg.max_no_accel_velocity_mm_s[1]), std::min(goto_default_feedrate, cfg.max_velocity_mm_s[1])}},
        {'Z', {(direction_value==0.0)?300:direction_value, std::min(5.0, cfg.max_no_accel_velocity_mm_s[2]), std::min(goto_default_feedrate, cfg.max_velocity_mm_s[2])}}};

    auto [distance, min_feed, max_feed] = directions[axis_id];
//    std::cerr << "DEBUG: " << direction_value << " " <<  distance << " " <<  min_feed << " " << max_feed << std::endl;
    auto backward_distance = -3.0 * std::abs(distance) / distance;
    // forward fast move. We will break it on endstop hit
    raspigcd::hardware::multistep_commands_t forward_fast_commands = program_to_steps(
        {{{'G', 1.0}, 
          {axis_id, std::min(std::abs(distance), 10.0) * std::abs(distance) / distance}, {'F', max_feed}},
          {{'G', 1.0}, {axis_id, distance}, {'F', max_feed}}},
        cfg, *(machine.motor_layout_.get()), {{'X', 0.0}, {'Y', 0.0}, {'Z', 0.0}, {'A', 0.0}, {'F', min_feed}}, [](const gcd::block_t) {});
    // backward slow move. We will break it on endstop release
    raspigcd::hardware::multistep_commands_t backward_slow_commands = program_to_steps(
        {{{'G', 1.0}, {axis_id, backward_distance * 0.5}, {'F', std::min(10.0, max_feed)}},
            {{'G', 1.0}, {axis_id, backward_distance}, {'F', std::min(1.0, min_feed)}}},
        cfg, *(machine.motor_layout_.get()), {{'X', 0.0}, {'Y', 0.0}, {'Z', 0.0}, {'A', 0.0}, {'F', min_feed}}, [](const gcd::block_t) {});

    std::atomic<bool> going_to_origin = true;
    auto probe_actions = [&going_to_origin, &machine](int k, int v) {
        std::cout << "[II] probe ... " << k << "  " << v << std::endl;
        if (going_to_origin) {
            if (v == 1) {
                going_to_origin = false;
                machine.stepping->terminate(20);
                std::cout << "[II] hit endstop to origin..." << std::endl;
            }
        } else {
            if (v == 0) {
                going_to_origin = true;
                machine.stepping->terminate(100);
                std::cout << "[II] released endstop to origin..." << std::endl;
            }
        }
    };

    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, probe_actions);
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, probe_actions);
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, probe_actions);

    try {
        machine.stepping->exec(forward_fast_commands, [machine, &is_cancel_execution, &is_paused, last_spindle_on_delay, &spindles_status](auto, auto tick_n) -> int {
            std::cout << "[II] terminated endstop at " << tick_n << " ticks." << std::endl;
            machine.timer_drv->wait_us(100000);
            if (is_cancel_execution) {
                for (auto e : spindles_status) {
                    machine.spindles_drv->spindle_pwm_power(e.first, 0);
                }
            }
            return 0; // stop execution
        });
    } catch (...) {
    }
    machine.timer_drv->wait_us(250000);
    if (!going_to_origin) {
        try {
            machine.stepping->exec(backward_slow_commands, [machine, &is_cancel_execution, &is_paused, last_spindle_on_delay, &spindles_status](auto, auto tick_n) -> int {
                std::cout << "[II] terminated endstop free at " << tick_n << std::endl;
                machine.timer_drv->wait_us(100000);
                if (is_cancel_execution) {
                    for (auto e : spindles_status) {
                        machine.spindles_drv->spindle_pwm_power(e.first, 0);
                    }
                }
                return 0; // stop execution
            });
        } catch (...) {
        }
    }
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, [](auto, auto) {});// ignore the endstop
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, [](auto, auto) {});// ignore the endstop
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, [](auto, auto) {});// ignore the endstop
    machine.stepping->exec(backward_slow_commands, [machine, &is_cancel_execution, &is_paused, last_spindle_on_delay, &spindles_status](auto, auto /*tick_n*/) -> int {
        if (is_cancel_execution) {
            for (auto e : spindles_status) {
                machine.spindles_drv->spindle_pwm_power(e.first, 0);
            }
        }
        return 0; // stop execution
    });
}

template <class T>
class fifo_c
{
    std::atomic_flag lock;
    std::list<T> data;

public:
    fifo_c<T>() : lock(ATOMIC_FLAG_INIT){};
    T get(std::atomic<bool>& cancel_execution)
    {
        while (!cancel_execution) {
            while (lock.test_and_set(std::memory_order_acquire))
                ;
            if (data.size() > 0) {
                auto ret = data.front();
                data.pop_front();
                lock.clear(std::memory_order_release);
                return ret;
            } else {
                lock.clear(std::memory_order_release);
                std::cerr << "X";
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        throw std::invalid_argument("fifo_c: the get from front broken.");
    }

    void put(std::atomic<bool>& cancel_execution, T value, int max_queue_size = 3)
    {
        while (!cancel_execution) {
            while (lock.test_and_set(std::memory_order_acquire))
                ;
            if ((int)data.size() < (int)max_queue_size) {
                data.push_back(value);
                lock.clear(std::memory_order_release);
                return;
            } else {
                lock.clear(std::memory_order_release);
                std::this_thread::sleep_for(std::chrono::milliseconds(40));
            }
        }

        throw std::invalid_argument("fifo_c: the put method broken.");
    }
};
/**
 * @brief produces series of multistep steps series filling the buffer that is a list of multistep commands. It can be canceled by setting cancel_execution to true.
 * 
 */
auto multistep_producer_for_execution = [](fifo_c<std::pair<hardware::multistep_commands_t, block_t>>& calculated_multisteps,
                                            partitioned_program_t& program_parts,
                                            execution_objects_t machine,
                                            converters::program_to_steps_f_t program_to_steps,
                                            std::atomic<bool>& cancel_execution,
                                            configuration::global cfg,
                                            block_t machine_state) -> int { // calculate multisteps
    std::map<int, double> spindles_status;

    for (std::size_t command_block_index = 0; (command_block_index < program_parts.size()) && (!cancel_execution); command_block_index++) {
        auto& ppart = program_parts[command_block_index];
        if (ppart.size() != 0) {
            if (ppart[0].count('M') == 0) {
                switch ((int)(ppart[0].at('G'))) {
                case 0:
                case 1: {
                    auto machine_state_prev = machine_state;

                    auto time0 = std::chrono::high_resolution_clock::now();
                    block_t st = last_state_after_program_execution(ppart, machine_state);
                    //// std::cout << "program_to_steps ... " << back_to_gcode({ppart}) << std::endl;
                    auto m_commands = program_to_steps(ppart, cfg, *(machine.motor_layout_.get()),
                        machine_state, [&machine_state](const gcd::block_t result) {
                            machine_state = result;
                        });

                    if (!(block_to_distance_with_v_t(st) == block_to_distance_with_v_t(machine_state))) {
                        std::cout << "states differs: " << block_to_distance_with_v_t(st) << "!=" << block_to_distance_with_v_t(machine_state) << std::endl;
                        throw std::invalid_argument("states differ");
                    }

                    auto time1 = std::chrono::high_resolution_clock::now();
                    double dt = std::chrono::duration<double, std::milli>(time1 - time0).count();
                    std::cout << "calculations of " << ppart.size() << " commands took " << dt << " milliseconds; have " << m_commands.size() << " steps to execute" << std::endl;
                    calculated_multisteps.put(cancel_execution, {m_commands, machine_state}, cfg.sequential_gcode_execution ? 1 : 5);

                    if (cancel_execution) return -100;
                } break;
                case 28: {
                    if (ppart[0].count('X')) machine_state['X'] = 0.0;
                    if (ppart[0].count('Y')) machine_state['Y'] = 0.0;
                    if (ppart[0].count('Z')) machine_state['Z'] = 0.0;
                    calculated_multisteps.put(cancel_execution, {{}, machine_state}, cfg.sequential_gcode_execution ? 1 : 5);
                    if (cancel_execution) return -100;
                } break;
                case 92: {
                    for (auto pelem : ppart) {
                        if (pelem.count('X')) machine_state['X'] = pelem['X'];
                        if (pelem.count('Y')) machine_state['Y'] = pelem['Y'];
                        if (pelem.count('Z')) machine_state['Z'] = pelem['Z'];
                    }
                    calculated_multisteps.put(cancel_execution, {{}, machine_state}, cfg.sequential_gcode_execution ? 1 : 5);
                    if (cancel_execution) return -100;
                } break;
                }
            } else {
                for (auto& m : ppart) {
                    switch ((int)(m.at('M'))) {
                    case 17:
                        break;
                    case 18:
                        break;
                    case 3:
                        spindles_status[0] = 1.0;
                        break;
                    case 5:
                        spindles_status[0] = 0.0;
                        break;
                    }
                }
            }
        }
    }
    return 0;
};


std::pair<int, block_t> execute_command_parts(partitioned_program_t program_parts,
    execution_objects_t machine,
    converters::program_to_steps_f_t program_to_steps,
    configuration::global cfg,
    std::atomic<bool>& cancel_execution,
    block_t machine_state_0)
{
    machine.steppers_drv->set_steps(machine.motor_layout_->cartesian_to_steps(block_to_distance_t(machine_state_0)));
    std::cout << "execute_command_parts: starting with steps counters: " << machine.steppers_drv->get_steps() << std::endl;
    fifo_c<std::pair<hardware::multistep_commands_t, block_t>> calculated_multisteps;

    std::atomic<bool> paused{false};
    std::function<void(int, int)> on_pause_execution = [machine, &paused](int, int s) {
        if (s == 1) {
            if (paused) {
                paused = false;
            } else {
                paused = true;
                machine.stepping->terminate(1000);
            }
        }
    };

    auto on_stop_execution = [machine, &program_parts, &cancel_execution](int k, int s) {
        if (s == 1) {
            std::cout << "on_stop_execution " << k << "  " << s << std::endl;
            cancel_execution = true;
            machine.stepping->terminate(1000);
        }
    };

    machine.buttons_drv->on_key(low_buttons_default_meaning_t::PAUSE, on_pause_execution);
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::TERMINATE, on_stop_execution);


    auto multistep_calculation_promise = std::async(std::launch::async, [&]() {
        //std::cout << "multistep_producer_for_execution start" << std::endl;
        try {
            auto ret = multistep_producer_for_execution(
                calculated_multisteps,
                program_parts,
                machine,
                program_to_steps,
                cancel_execution,
                cfg,
                machine_state_0);
            //std::cout << "multistep_producer_for_execution finished with " << ret << " code" << std::endl;
            return ret;
        } catch (std::invalid_argument& e) {
            std::cout << "multistep_producer_for_execution terminated while generating next parts of execution" << std::endl;
            return -200;
        }
    });


    {
        block_t machine_state_ret = machine_state_0;

        std::map<int, double> spindles_status;
        long int last_spindle_on_delay = 7000;

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
        for (std::size_t command_block_index = 0; (command_block_index < program_parts.size()) && (!cancel_execution); command_block_index++) {
            auto& ppart = program_parts[command_block_index];


            if (ppart.size() != 0) {
                if (ppart[0].count('M') == 0) {
                    switch ((int)(ppart[0].at('G'))) {
                    case 0:
                    case 1: {
                        auto [m_commands, machine_state] = calculated_multisteps.get(cancel_execution);
                        try {
                            if (((int)(ppart[0].at('G')) == 1) && cfg.spindles.at(0).mode == configuration::spindle_modes::LASER) {
                                machine.spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                            }
                            execute_calculated_multistep(m_commands, machine, on_stop_execution, cancel_execution, paused, last_spindle_on_delay, spindles_status);
                            if (((int)(ppart[0].at('G')) == 1) && cfg.spindles.at(0).mode == configuration::spindle_modes::LASER) {
                                machine.spindles_drv->spindle_pwm_power(0, 0.0);
                            }
                            machine_state_ret = machine_state;
                        } catch (const raspigcd::hardware::execution_terminated& et) {
                            machine.spindles_drv->spindle_pwm_power(0, 0.0);
                            //                            machine.steppers_drv->enable_steppers({false,false,false,false});
                            //std::cerr << "TERMINATED at " << currstate << std::endl;
                            auto currstate = block_to_distance_with_v_t(machine_state);
                            auto end_pos = machine.motor_layout_->steps_to_cartesian(machine.steppers_drv->get_steps());
                            for (unsigned i = 0; i < end_pos.size(); i++)
                                currstate[i] = end_pos[i];
                            return {-2, distance_with_velocity_to_block(currstate)};
                        }
                    } break;
                    case 28: {
                        auto [m_commands, machine_state] = calculated_multisteps.get(cancel_execution);
                        for (auto pelem : ppart) {
                            if ((int)(pelem.count('X'))) {
                                home_position_find('X',
                                pelem['X'], 
                                program_to_steps, cfg, machine, cancel_execution, paused, 
                                last_spindle_on_delay, spindles_status,pelem.count('F')?pelem['F']:50);
                            }
                            if ((int)(pelem.count('Y'))) {
                                home_position_find('Y', 
                                pelem['Y'],
                                program_to_steps, cfg, machine, cancel_execution, paused, 
                                last_spindle_on_delay, spindles_status,pelem.count('F')?pelem['F']:50);
                            }
                            if ((int)(pelem.count('Z'))) {
                                home_position_find('Z', 
                                pelem['Z']
                                ,program_to_steps, cfg, machine, cancel_execution, paused, 
                                last_spindle_on_delay, spindles_status,pelem.count('F')?pelem['F']:50);
                            }
                        }
                        machine_state_ret = machine_state;
                    } break;
                    case 92: {
                        auto [m_commands, machine_state] = calculated_multisteps.get(cancel_execution);
                        auto position_from_steps = machine.motor_layout_->steps_to_cartesian(machine.steppers_drv->get_steps());
                        for (auto pelem : ppart) {
                            if ((int)(pelem.count('X'))) {
                                position_from_steps[0] = pelem['X'];
                            }
                            if ((int)(pelem.count('Y'))) {
                                position_from_steps[1] = pelem['Y'];
                            }
                            if ((int)(pelem.count('Z'))) {
                                position_from_steps[2] = pelem['Z'];
                            }
                        }
                        machine.steppers_drv->set_steps(machine.motor_layout_->cartesian_to_steps(position_from_steps));
                        machine_state['X'] = position_from_steps[0];
                        machine_state['Y'] = position_from_steps[1];
                        machine_state['Z'] = position_from_steps[2];
                        machine_state_ret = machine_state;
                    } break;
                    }
                } else {
                    for (auto& m : ppart) {
                        switch ((int)(m.at('M'))) {
                        case 17:
                            machine.steppers_drv->enable_steppers({true, true, true, true});
                            wait_for_component_to_start(m, 200);
                            break;
                        case 18:
                            machine.steppers_drv->enable_steppers({false, false, false, false});
                            wait_for_component_to_start(m, 200);
                            break;
                        case 3:
                            spindles_status[0] = 1.0;
                            if (cfg.spindles.at(0).mode != configuration::spindle_modes::LASER) {
                                machine.spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                            }
                            last_spindle_on_delay = wait_for_component_to_start(m, 3000);
                            break;
                        case 5:
                            spindles_status[0] = 0.0;
                            if (cfg.spindles.at(0).mode != configuration::spindle_modes::LASER) {
                                machine.spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                            }
                            wait_for_component_to_start(m, 3000);
                            break;
                        }
                    }
                }
            }
        }
        return {multistep_calculation_promise.get(), machine_state_ret};
    }
};


auto execute_gcode_text = [](const configuration::global cfg, const bool raw_gcode, const auto gcode_text, const auto& machine, std::atomic<bool>& cancel_execution, block_t machine_state_0 = {{'F', 0.5}}) {
    converters::program_to_steps_f_t program_to_steps = converters::program_to_steps_factory(cfg.steps_generator);

    auto program = gcode_to_maps_of_arguments(gcode_text);
    //            std::cout << "PRORGRAM RAW: \n" << back_to_gcode({program}) << std::endl;
    program = enrich_gcode_with_feedrate_commands(std::move(program), cfg);
    //            std::cout << back_to_gcode({program}) << std::endl;
    // program = remove_g92_from_gcode(program);
    if (!raw_gcode) {
        program = optimize_path_douglas_peucker(program, cfg.douglas_peucker_marigin, machine_state_0);
    }
    auto program_parts = group_gcode_commands(std::move(program));

    block_t machine_state = machine_state_0; //{{'F', 0.5}};
    if (!raw_gcode) {
        std::cerr << "PREPROCESSING GCODE" << std::endl;
        program_parts = insert_additional_nodes_inbetween(program_parts, machine_state, cfg);
        //std::cerr << back_to_gcode(program_parts) << std::endl;
        machine_state['F'] = *std::min_element(cfg.max_no_accel_velocity_mm_s.begin(), cfg.max_no_accel_velocity_mm_s.end());
        program_parts = preprocess_program_parts(program_parts, cfg, machine_state);
        //std::cerr << back_to_gcode(program_parts) << std::endl;
    } // if prepare paths

    //std::cerr << "STARTING...." << std::endl;

    return execute_command_parts(std::move(program_parts), machine, program_to_steps, cfg, cancel_execution, machine_state_0);
};

/**
 * @brief execute the gcode file for the given machine, with the given starting point and for the specific configuration.
 * 
 */
auto execute_gcode_file = [](const configuration::global cfg, const bool raw_gcode, const auto filename, const auto& machine, std::atomic<bool>& cancel_execution, block_t machine_state_0 = {{'F', 0.5}}) {
    using namespace raspigcd;
    using namespace raspigcd::hardware;

    std::ifstream gcd_file(filename);
    if (!gcd_file.is_open()) throw std::invalid_argument("could not open file \"" + filename + "\"");
    std::string gcode_text((std::istreambuf_iterator<char>(gcd_file)),
        std::istreambuf_iterator<char>());
    return execute_gcode_text(cfg, raw_gcode, gcode_text, machine, cancel_execution, machine_state_0);
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


auto interactive_mode_execution = [](const auto cfg, const auto raw_gcode) {
    using namespace raspigcd;
    using namespace raspigcd::hardware;

    auto machine = stepping_simple_timer_factory(cfg);

    converters::program_to_steps_f_t program_to_steps;
    program_to_steps = converters::program_to_steps_factory(cfg.steps_generator);


    machine.buttons_drv->on_key(low_buttons_default_meaning_t::PAUSE, [](int k, int v) { std::cout << "PAUSE     " << k << "  value=" << v << std::endl; });
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::TERMINATE, [](int k, int v) { std::cout << "TERMINATE " << k << "  value=" << v << std::endl; });
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, [](int k, int v) { std::cout << "ENDSTOP_X " << k << "  value=" << v << std::endl; });
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, [](int k, int v) { std::cout << "ENDSTOP_Y " << k << "  value=" << v << std::endl; });
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, [](int k, int v) { std::cout << "ENDSTOP_Z " << k << "  value=" << v << std::endl; });


    std::cout << "type 'q' to quit" << std::endl;
    std::string command;
    std::future<block_t> execute_promise;
    block_t machine_status_after_exec = {{'F', 0.5}};
    std::atomic<bool> cancel_execution = false;

    do {
        using namespace std::chrono_literals;
        std::cin >> command;
        if (execute_promise.valid() && (execute_promise.wait_for(10ms) == std::future_status::ready)) {
            try {
                machine_status_after_exec = execute_promise.get();
            } catch (const std::invalid_argument& e) {
                std::cerr << "EXECUTION_FAILED: " << e.what() << std::endl;
            }
        }
        if ((command == "execute") || (command == "exec")) {
            std::string filename;
            std::getline(std::cin, filename);
            filename = std::regex_replace(filename, std::regex("^ +"), "");
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
                });
            }
        } else if ((command == "sim_execute") || (command == "sim_exec")) {
            std::string filename;
            std::getline(std::cin, filename);
            filename = std::regex_replace(filename, std::regex("^ +"), "");
            if (execute_promise.valid()) {
                std::cout << "task is currently executing..." << std::endl;
            } else {
                cancel_execution = false;
                try {
                    double executio_time = fake_execution_and_statistics_collect(cfg, [&](execution_objects_t& machine) {
                        auto [err_code, machine_state] = execute_gcode_file(cfg, raw_gcode, filename, machine, cancel_execution, machine_status_after_exec);
                        if (err_code != 0) std::cerr << "ERROR_AFTER_EXECUTION: " << err_code << std::endl;
                    });
                    std::cout << "SIM_EXEC_TIME: " << executio_time << std::endl;
                } catch (std::exception& e) {
                    std::cout << "SIM_EXEC_TIME: "
                              << "ERROR: " << e.what() << std::endl;
                }
            }
        } else if (command == "sim_go") {
            std::string gcdcommand;
            std::getline(std::cin, gcdcommand);
            gcdcommand = std::regex_replace(gcdcommand, std::regex("^ +"), "");
            gcdcommand = std::regex_replace(gcdcommand, std::regex("[\\\\][n]"), "\n");
            //std::cerr << gcdcommand;
            if (execute_promise.valid()) {
                std::cout << "task is currently executing..." << std::endl;
            } else {
                cancel_execution = false;
                try {
                    double executio_time = fake_execution_and_statistics_collect(cfg, [&](execution_objects_t& machine) {
                        auto [err_code, machine_state] = execute_gcode_text(cfg, raw_gcode, gcdcommand + "\n", machine, cancel_execution, machine_status_after_exec);
                        if (err_code != 0) std::cerr << "ERROR_AFTER_EXECUTION: " << err_code << std::endl;
                    });
                    std::cout << "SIM_GO_TIME: " << executio_time << std::endl;
                } catch (std::exception& e) {
                    std::cout << "SIM_GO_TIME: "
                              << "ERROR: " << e.what() << std::endl;
                }
            }
        } else if (command == "go") {
            std::string gcdcommand;
            std::getline(std::cin, gcdcommand);
            gcdcommand = std::regex_replace(gcdcommand, std::regex("^ +"), "");
            gcdcommand = std::regex_replace(gcdcommand, std::regex("[\\\\][n]"), "\n");
            //std::cerr << gcdcommand;
            if (execute_promise.valid()) {
                std::cout << "task is currently executing..." << std::endl;
            } else {
                cancel_execution = false;
                execute_promise = std::async([&, gcdcommand]() {
                    low_buttons_handlers_guard low_buttons_handlers_guard_(machine.buttons_drv);
                    auto [err_code, machine_state] = execute_gcode_text(cfg, raw_gcode, gcdcommand + "\n", machine, cancel_execution, machine_status_after_exec);
                    auto end_pos = machine.motor_layout_->steps_to_cartesian(machine.steppers_drv->get_steps());
                    machine_state['X'] = end_pos[0];
                    machine_state['Y'] = end_pos[1];
                    machine_state['Z'] = end_pos[2];
                    if (err_code == 0)
                        std::cout << "EXECUTE_DONE: " << end_pos << std::endl;
                    else
                        std::cout << "EXECUTE_DONE_ERROR: " << end_pos << std::endl;

                    return machine_state;
                });
            }
        } else if (command == "stop") {
            cancel_execution = true;
            machine.stepping->terminate(1000);
            try {
                machine_status_after_exec = execute_promise.get();
            } catch (const std::invalid_argument& e) {
                std::cerr << "EXECUTION_FAILED: " << e.what() << std::endl;
            }
            // TODO: go to base
            cancel_execution = false;
            low_buttons_handlers_guard low_buttons_handlers_guard_(machine.buttons_drv);
            auto [err_code, machine_state] = execute_gcode_text(cfg, raw_gcode, "G0Z10\nG0X0Y0\nG0Z0\n", machine, cancel_execution, machine_status_after_exec);
            auto end_pos = machine.motor_layout_->steps_to_cartesian(machine.steppers_drv->get_steps());
            machine_status_after_exec = machine_state;
            machine_status_after_exec['X'] = end_pos[0];
            machine_status_after_exec['Y'] = end_pos[1];
            machine_status_after_exec['Z'] = end_pos[2];
            if (err_code == 0)
                std::cout << "EXECUTE_DONE: " << end_pos << std::endl;
            else
                std::cout << "EXECUTE_DONE_ERROR: " << end_pos << std::endl;

            machine.steppers_drv->enable_steppers({false, false, false, false});
            std::cout << "STOPPED: " << end_pos << std::endl;
        } else if (command == "terminate") {
            cancel_execution = true;
            machine.stepping->terminate(1000);
            try {
                machine_status_after_exec = execute_promise.get();
            } catch (const std::invalid_argument& e) {
                std::cerr << "EXECUTION_FAILED: " << e.what() << std::endl;
            }
            auto end_pos = machine.motor_layout_->steps_to_cartesian(machine.steppers_drv->get_steps());
            std::cout << "TERMINATED: " << end_pos << std::endl;
        } else if (command == "q") {
        } else if (command == "status") {
            auto end_steps = machine.steppers_drv->get_steps();
            auto end_pos = machine.motor_layout_->steps_to_cartesian(end_steps);
            std::cout << "STEPS: ";
            for (auto v : end_steps)
                std::cout << v << " ";
            if (execute_promise.valid() && (execute_promise.wait_for(10ms) != std::future_status::ready)) {
                std::cout << "STATUS: " << end_pos << " running" << std::endl;
            } else {
                std::cout << "STATUS_ELEMENT: ";
                for (auto [k, v] : machine_status_after_exec)
                    std::cout << " " << k << "=" << v;
                std::cout << std::endl;
                std::cout << "STATUS: " << end_pos << " idle" << std::endl;
            }
        } else {
            std::cout << "UNKNOWN_COMMAND: " << command << std::endl;
            std::cout << "INFO: Valid commands are:" << std::endl;
            std::cout << "INFO:  q                     -> quit" << std::endl;
            std::cout << "INFO:  go [g-code]           -> execute gcode command" << std::endl;
            std::cout << "INFO:  exec [filename]       -> execute gcode file" << std::endl;
            std::cout << "INFO:  sim_go [g-code]       -> simulate execution of gcode command" << std::endl;
            std::cout << "INFO:  sim_exec [filename]   -> simulate execution of gcode file" << std::endl;
            std::cout << "INFO:  status                -> get status and last position" << std::endl;
            std::cout << "INFO:  stop                  -> stop and go to origin" << std::endl;
            std::cout << "INFO:  terminate             -> terminate current execution" << std::endl;
        }
    } while (command != "q");
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
        } else if (args.at(i) == "-f") {
            i++;
            auto machine = stepping_simple_timer_factory(cfg);
            std::atomic<bool> cancel_execution = false;
            execute_gcode_file(cfg, raw_gcode, args.at(i), machine, cancel_execution);
        } else if (args.at(i) == "--configtest") {
            interactive_mode_execution(cfg, raw_gcode);
            i++;
        }
    }

    return 0;
}
