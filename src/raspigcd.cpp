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
    std::cout << "\t\tTreat the file as raw - no additional processing. No limits check." << std::endl;
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
    std::cout << "\tCopyright (C) 2019  Tadeusz Puźniakowski puzniakowski.pl" << std::endl;
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


partitioned_program_t preprocess_program_parts(partitioned_program_t program_parts, const configuration::global& cfg)
{
    block_t machine_state = {{'F', *std::min_element(cfg.max_no_accel_velocity_mm_s.begin(), cfg.max_no_accel_velocity_mm_s.end())}};
    program_t prepared_program;

    for (auto& ppart : program_parts) {
        if (ppart.size() != 0) {
            if (ppart[0].count('M') == 0) {
                //std::cout << "G PART: " << ppart.size() << std::endl;
                switch ((int)(ppart[0]['G'])) {
                case 0:
                case 1:
                    ppart = g1_move_to_g1_with_machine_limits(ppart, cfg, machine_state);
                    prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                    machine_state = last_state_after_program_execution(ppart, machine_state);
                    break;
                case 4:
                case 28:
                case 92:
                    prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
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

    //if (save_to_files_list.size() > 0) {
    //    std::cout << "SAVING prepared_program without DP FILE TO: " << (save_to_files_list.front()+".stage3") << std::endl;
    //    std::fstream f (save_to_files_list.front()+".stage3", std::fstream::out);
    //    f << back_to_gcode(group_gcode_commands(prepared_program)) << std::endl;
    //}
    prepared_program = optimize_path_douglas_peucker(prepared_program, cfg.douglas_peucker_marigin);
    program_parts = group_gcode_commands(remove_duplicate_blocks(prepared_program, {}));
    machine_state = {{'F', 0.5}};
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
            using namespace std::chrono_literals;
            machine.timer_drv->wait_us(1000 * last_spindle_on_delay);
        }
        return 1; // continue execuiton
    });
}

void home_position_find(char axis_id, converters::program_to_steps_f_t program_to_steps, configuration::global cfg, execution_objects_t machine, std::atomic<bool>& cancel_execution, std::atomic<bool>& paused, long int last_spindle_on_delay, std::map<int, double>& spindles_status)
{
    std::function<void(int, int)> on_stop_execution = [](int, int) {};


    static std::map<char, std::tuple<double, double, double>> directions = {
        {'X', {-2000, cfg.max_no_accel_velocity_mm_s[0], cfg.max_velocity_mm_s[0]}},
        {'Y', {6000, cfg.max_no_accel_velocity_mm_s[1], cfg.max_velocity_mm_s[1]}},
        {'Z', {200, cfg.max_no_accel_velocity_mm_s[2], cfg.max_velocity_mm_s[2]}}};


    auto [distance, max_feed, min_feed] = directions[axis_id];
    auto backward_distance = -1000.0 * std::abs(distance) / distance;
    raspigcd::hardware::multistep_commands_t forward_fast_commands = program_to_steps(
        {{{'G', 0.0}, {axis_id, distance * 0.05}, {'F', min_feed}},
            {{'G', 0.0}, {axis_id, distance}, {'F', max_feed}}},
        cfg, *(machine.motor_layout_.get()), {{'X', 0.0}, {'Y', 0.0}, {'Z', 0.0}, {'A', 0.0}, {'F', min_feed}}, [](const gcd::block_t) {});
    raspigcd::hardware::multistep_commands_t backward_slow_commands = program_to_steps(
        {{{'G', 0.0}, {axis_id, 0.0}, {'F', min_feed}},
            {{'G', 0.0}, {axis_id, backward_distance}, {'F', min_feed}}},
        cfg, *(machine.motor_layout_.get()), {{'X', 0.0}, {'Y', 0.0}, {'Z', 0.0}, {'A', 0.0}, {'F', min_feed}}, [](const gcd::block_t) {});

    std::atomic<bool> going_to_origin = true;
    auto probe_actions = [&going_to_origin, &machine](int k, int v) {
        if (going_to_origin) {
            if (v == 1) {
                going_to_origin = false;
                machine.stepping->terminate(1500);
                std::cout << "hit endstop..." << std::endl;
            }
        } else {
            std::cout << "released endstop ? ... " << k << "  " << v << std::endl;
            if (v == 0) {
                going_to_origin = true;
                machine.stepping->terminate(1000);
                std::cout << "released endstop..." << std::endl;
            }
        }
    };

    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, probe_actions);
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, probe_actions);
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, probe_actions);

    try {
        machine.stepping->exec(forward_fast_commands, [machine, &cancel_execution, &paused, last_spindle_on_delay, &spindles_status](auto, auto tick_n) -> int {
            std::cout << "terminated endstop at " << tick_n << std::endl;
            machine.timer_drv->wait_us(100000);
            if (cancel_execution) {
                for (auto e : spindles_status) {
                    machine.spindles_drv->spindle_pwm_power(e.first, 0);
                }
            }
            return 0; // stop execution
        });
    } catch (...) {
    }
    try {
        machine.stepping->exec(backward_slow_commands, [machine, &cancel_execution, &paused, last_spindle_on_delay, &spindles_status](auto, auto tick_n) -> int {
            std::cout << "terminated endstop free at " << tick_n << std::endl;
            machine.timer_drv->wait_us(100000);
            if (cancel_execution) {
                for (auto e : spindles_status) {
                    machine.spindles_drv->spindle_pwm_power(e.first, 0);
                }
            }
            return 0; // stop execution
        });
    } catch (...) {
    }
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_X, [](auto, auto) {});
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Y, [](auto, auto) {});
    machine.buttons_drv->on_key(low_buttons_default_meaning_t::ENDSTOP_Z, [](auto, auto) {});
}


auto push_multisteps = [](std::list<std::pair<hardware::multistep_commands_t, block_t>>& calculated_multisteps, std::mutex& calculated_multisteps_m, std::atomic<bool>& cancel_execution, std::pair<hardware::multistep_commands_t, block_t> multisteps_pair) {
    while (!cancel_execution) {
        calculated_multisteps_m.lock();
        if (calculated_multisteps.size() < 2) {
            calculated_multisteps.push_back(std::move(multisteps_pair));
            calculated_multisteps_m.unlock();
            break;
        }
        calculated_multisteps_m.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
};
auto pop_multisteps = [](std::list<std::pair<hardware::multistep_commands_t, block_t>>& calculated_multisteps, std::mutex& calculated_multisteps_m, std::atomic<bool>& cancel_execution) -> std::pair<hardware::multistep_commands_t, block_t> {
    while (true) {
        calculated_multisteps_m.lock();
        if (calculated_multisteps.size() > 0) {
            calculated_multisteps_m.unlock();
            break;
        }
        calculated_multisteps_m.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }
    calculated_multisteps_m.lock();
    auto [m_commands, machine_state] = calculated_multisteps.front();
    calculated_multisteps.pop_front();
    calculated_multisteps_m.unlock();
    return {m_commands, machine_state};
};

/**
 * @brief produces series of multistep steps series filling the buffer that is a list of multistep commands. It can be canceled by setting cancel_execution to true.
 * 
 */
auto multistep_producer_for_execution = [](std::list<std::pair<hardware::multistep_commands_t, block_t>>& calculated_multisteps,
                                            std::mutex& calculated_multisteps_m,
                                            partitioned_program_t &program_parts,
                                            execution_objects_t machine,
                                            converters::program_to_steps_f_t program_to_steps,
                                            std::atomic<bool>& cancel_execution,
                                            configuration::global cfg) -> int { // calculate multisteps
    block_t machine_state = {{'F', 0.5}};

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
                    std::cout << "calculations took " << dt << " milliseconds; have " << m_commands.size() << " steps to execute" << std::endl;
                    push_multisteps(calculated_multisteps, calculated_multisteps_m, cancel_execution, {m_commands, machine_state});

                    if (cancel_execution) return -100;
                } break;
                case 28: {
                    if (ppart[0].count('X')) machine_state['X'] = 0.0;
                    if (ppart[0].count('Y')) machine_state['Y'] = 0.0;
                    if (ppart[0].count('Z')) machine_state['Z'] = 0.0;
                    push_multisteps(calculated_multisteps, calculated_multisteps_m, cancel_execution, {{}, machine_state});
                    if (cancel_execution) return -100;
                } break;
                case 92: {
                    for (auto pelem : ppart) {
                        if (pelem.count('X')) machine_state['X'] = pelem['X'];
                        if (pelem.count('Y')) machine_state['Y'] = pelem['Y'];
                        if (pelem.count('Z')) machine_state['Z'] = pelem['Z'];
                    }
                    push_multisteps(calculated_multisteps, calculated_multisteps_m, cancel_execution, {{}, machine_state});
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


int execute_command_parts(partitioned_program_t program_parts, execution_objects_t machine, converters::program_to_steps_f_t program_to_steps, configuration::global cfg)
{
    std::list<std::pair<hardware::multistep_commands_t, block_t>> calculated_multisteps;
    std::mutex calculated_multisteps_m;
    std::atomic<bool> cancel_execution = false;


    auto multistep_calculation_promise = std::async(std::launch::async, [&](){return multistep_producer_for_execution( 
    calculated_multisteps,
    calculated_multisteps_m,
    program_parts,
    machine,
    program_to_steps,
    cancel_execution,
    cfg);});


    {
        block_t machine_state = {{'F', 0.5}};

        std::map<int, double> spindles_status;
        long int last_spindle_on_delay = 7000;

        for (std::size_t command_block_index = 0; (command_block_index < program_parts.size()) && (!cancel_execution); command_block_index++) {
            auto& ppart = program_parts[command_block_index];

            std::atomic<bool> paused = false;
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


            if (ppart.size() != 0) {
                if (ppart[0].count('M') == 0) {
                    switch ((int)(ppart[0].at('G'))) {
                    case 0:
                    case 1: {
                        auto [m_commands, machine_state] = pop_multisteps(calculated_multisteps, calculated_multisteps_m, cancel_execution);
                        try {
                            execute_calculated_multistep(m_commands, machine, on_stop_execution, cancel_execution, paused, last_spindle_on_delay, spindles_status);
                        } catch (const raspigcd::hardware::execution_terminated& et) {
                            machine.spindles_drv->spindle_pwm_power(0, 0.0);
                            machine.steppers_drv->enable_steppers({false});
                            auto currstate = block_to_distance_with_v_t(machine_state);
                            auto ddp = machine.motor_layout_->steps_to_cartesian(et.delta_steps);
                            for (int i = 0; i < 3; i++)
                                currstate[i] += ddp[i];
                            currstate[3] = 0.001;
                            std::cerr << "TERMINATED at " << currstate << std::endl;
                            machine_state = distance_with_velocity_to_block(currstate);
                            return -2;
                        }
                    } break;
                    case 28: {
                        auto [m_commands, machine_state] = pop_multisteps(calculated_multisteps, calculated_multisteps_m, cancel_execution);
                        for (auto pelem : ppart) {
                            if ((int)(pelem.count('X'))) {
                                home_position_find('X', program_to_steps, cfg, machine, cancel_execution, paused, last_spindle_on_delay, spindles_status);
                            }
                            if ((int)(pelem.count('Y'))) {
                                home_position_find('Y', program_to_steps, cfg, machine, cancel_execution, paused, last_spindle_on_delay, spindles_status);
                            }
                            if ((int)(pelem.count('Z'))) {
                                home_position_find('Z', program_to_steps, cfg, machine, cancel_execution, paused, last_spindle_on_delay, spindles_status);
                            }
                        }
                    } break;
                    case 92: {
                        auto [m_commands, machine_state] = pop_multisteps(calculated_multisteps, calculated_multisteps_m, cancel_execution);
                    } break;
                    }
                } else {
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
                    for (auto& m : ppart) {
                        switch ((int)(m.at('M'))) {
                        case 17:
                            machine.steppers_drv->enable_steppers({true});
                            wait_for_component_to_start(m, 200);
                            break;
                        case 18:
                            machine.steppers_drv->enable_steppers({false});
                            wait_for_component_to_start(m, 200);
                            break;
                        case 3:
                            spindles_status[0] = 1.0;
                            machine.spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                            last_spindle_on_delay = wait_for_component_to_start(m, 3000);
                            break;
                        case 5:
                            spindles_status[0] = 0.0;
                            machine.spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                            wait_for_component_to_start(m, 3000);
                            break;
                        }
                    }
                }
            }
        }
        return multistep_calculation_promise.get();
    }
    
    return multistep_calculation_promise.get();
};


int main(int argc, char** argv)
{
    using namespace std::chrono_literals;
    std::vector<std::string> args(argv, argv + argc);
    configuration::global cfg;
    cfg.load_defaults();

    bool raw_gcode = false; // should I push G commands directly, without adaptation to machine
    std::list<std::string> save_to_files_list;
    for (unsigned i = 1; i < args.size(); i++) {
        if ((args.at(i) == "-h") || (args.at(i) == "--help")) {
            help_text(args);
        } else if (args.at(i) == "-c") {
            i++;
            cfg.load(args.at(i));
        } else if (args.at(i) == "-C") {
            std::cout << cfg << std::endl;
        } else if (args.at(i) == "-s") {
            i++;
            save_to_files_list.push_back(args.at(i));
        } else if (args.at(i) == "--raw") {
            raw_gcode = true;
        } else if (args.at(i) == "-f") {
            using namespace raspigcd;
            using namespace raspigcd::hardware;

            auto machine = stepping_simple_timer_factory(cfg);

            converters::program_to_steps_f_t program_to_steps;
            program_to_steps = converters::program_to_steps_factory(cfg.steps_generator);

            i++;
            std::ifstream gcd_file(args.at(i));
            if (!gcd_file.is_open()) throw std::invalid_argument("file should be opened");
            std::string gcode_text((std::istreambuf_iterator<char>(gcd_file)),
                std::istreambuf_iterator<char>());

            auto program = gcode_to_maps_of_arguments(gcode_text);
            program = enrich_gcode_with_feedrate_commands(std::move(program), cfg);
            //program = remove_g92_from_gcode(program);
            if (!raw_gcode) {
                program = optimize_path_douglas_peucker(program, cfg.douglas_peucker_marigin);
            }
            auto program_parts = group_gcode_commands(std::move(program));

            block_t machine_state = {{'F', 0.5}};
            if (!raw_gcode) {
                std::cerr << "preprocessing gcode" << std::endl;
                program_parts = insert_additional_nodes_inbetween(program_parts, machine_state, cfg);
                //std::cout << back_to_gcode(program_parts) << std::endl;
                program_parts = preprocess_program_parts(program_parts, cfg);
            } // if prepare paths

            std::cout << "STARTING...." << std::endl;

            if (save_to_files_list.size() > 0) {
                std::cout << "SAVING PREPROCESSED FILE TO: " << save_to_files_list.front() << std::endl;
                std::fstream f(save_to_files_list.front(), std::fstream::out);
                save_to_files_list.pop_front();
                f << back_to_gcode(program_parts) << std::endl;
            }

            execute_command_parts(std::move(program_parts), machine, program_to_steps, cfg);
        }
    }

    return 0;
}
