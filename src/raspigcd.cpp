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
                    //ppart = g0_move_to_g1_sequence(ppart, cfg, machine_state);
                    //for (auto &e : ppart) e['G'] = 0;
                    //prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                    //machine_state = last_state_after_program_execution(ppart,machine_state);
                    //break;
                case 1:
                    ppart = g1_move_to_g1_with_machine_limits(ppart, cfg, machine_state);
                    prepared_program.insert(prepared_program.end(), ppart.begin(), ppart.end());
                    machine_state = last_state_after_program_execution(ppart, machine_state);
                    break;
                case 4:
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

using low_level_components_t = std::tuple<
    std::shared_ptr<low_steppers>,
    std::shared_ptr<low_spindles_pwm>,
    std::shared_ptr<low_buttons>>;

// void execute_gcode_string(std::string &gcode_string,low_level_components_t llc) {
//     auto &[steppers_drv, spindles_drv,buttons_drv ] = llc;
//
//
// }

//partitioned_program_t preprocess_gcode() {
//
//}


program_t enrich_gcode_with_feedrate_commands(const program_t& program_, const configuration::global& cfg)
{
    auto program = program_;
    double previous_feedrate_g1 = 0.1;
    for (auto& p : program) {
        if (p.count('G')) {
            if (p['G'] == 0) {
                p['F'] = *std::max_element(
                    std::begin(cfg.max_velocity_mm_s),
                    std::end(cfg.max_velocity_mm_s));
            } else if (p['G'] == 1) {
                if (p.count('F')) {
                    previous_feedrate_g1 = p['F'];
                } else {
                    p['F'] = previous_feedrate_g1;
                }
            }
        }
    }
    return program;
}


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
            //bool enable_video = false;

            //std::shared_ptr<low_steppers> steppers_drv;
            //std::shared_ptr<low_spindles_pwm> spindles_drv;
            //std::shared_ptr<low_buttons> buttons_drv;


            //steps_t position_for_fake;


            //auto [stepping, motor_layout_]
            auto [timer_drv, steppers_drv,
                spindles_drv,
                buttons_drv,
                motor_layout_,
                stepping] = stepping_simple_timer_factory(cfg);

            std::atomic<int> break_execution_result = -1;
            std::function<void(int, int)> on_pause_execution;
            auto on_resume_execution = [&stepping, buttons_drv, &on_pause_execution, &break_execution_result](int k, int s) {
                if (s == 1) {
                    std::cout << "######## resume" << std::endl;
                    break_execution_result = 1;
                    buttons_drv->on_key(k, on_pause_execution);
                }
            };
            auto on_stop_execution = [&stepping, buttons_drv, &break_execution_result](int k, int s) {
                if (s == 1) {
                    break_execution_result = 0;
                    stepping->terminate(1000);
                }
            };
            on_pause_execution = [&stepping, buttons_drv, &on_resume_execution, &break_execution_result](int k, int s) {
                if ((s == 1) && (break_execution_result == -1)) {
                    break_execution_result = -1;
                    buttons_drv->on_key(k, on_resume_execution);
                    stepping->terminate(1000);
                }
            };

            buttons_drv->on_key(low_buttons_default_meaning_t::PAUSE, on_pause_execution);
            buttons_drv->on_key(low_buttons_default_meaning_t::TERMINATE, on_stop_execution);
            converters::program_to_steps_f_t program_to_steps;
            program_to_steps = converters::program_to_steps_factory(cfg.steps_generator);

            i++;
            std::ifstream gcd_file(args.at(i));
            if (!gcd_file.is_open()) throw std::invalid_argument("file should be opened");
            std::string gcode_text((std::istreambuf_iterator<char>(gcd_file)),
                std::istreambuf_iterator<char>());

            auto program = gcode_to_maps_of_arguments(gcode_text);
            program = enrich_gcode_with_feedrate_commands(program, cfg);
            program = remove_g92_from_gcode(program);
            if (!raw_gcode) {
                program = optimize_path_douglas_peucker(program, cfg.douglas_peucker_marigin);
            }
            auto program_parts = group_gcode_commands(program);
            block_t machine_state = {{'F', 0.5}};
            if (!raw_gcode) {
                program_parts = insert_additional_nodes_inbetween(program_parts, machine_state, cfg);
                program_parts = preprocess_program_parts(program_parts, cfg);
            } // if prepare paths

            std::cout << "STARTING...." << std::endl;

            if (save_to_files_list.size() > 0) {
                std::cout << "SAVING PREPROCESSED FILE TO: " << save_to_files_list.front() << std::endl;
                std::fstream f(save_to_files_list.front(), std::fstream::out);
                save_to_files_list.pop_front();
                f << back_to_gcode(program_parts) << std::endl;
            }
            machine_state = {{'F', 0.5}};
            std::map<int, double> spindles_status;
            long int last_spindle_on_delay = 7000;
            for (auto& ppart : program_parts) {
                //std::cout << "Put part: " << ppart.size() << std::endl;
                if (ppart.size() != 0) {
                    if (ppart[0].count('M') == 0) {
                        //std::cout << "G PART: " << ppart.size() << std::endl;
                        switch ((int)(ppart[0]['G'])) {
                        case 0:
                        case 1:
                            //  case 4:
                            //if (video.get() != nullptr) {
                            //    video->set_g_state((int)(ppart[0]['G']));
                            //}
                            auto machine_state_prev = machine_state;

                            auto time0 = std::chrono::high_resolution_clock::now();
                            block_t st = last_state_after_program_execution(ppart, machine_state);
                            auto m_commands = program_to_steps(ppart, cfg, *(motor_layout_.get()),
                                machine_state, [&machine_state](const block_t result) {
                                    machine_state = result;
                                    // std::cout << "____ program_to_steps:";
                                    // for (auto& s : machine_state) {
                                    //     std::cout << s.first << ":" << s.second << " ";
                                    // }
                                    // std::cout << std::endl;
                                });
                            if (!(block_to_distance_with_v_t(st) == block_to_distance_with_v_t(machine_state))) {
                                std::cout << "states differs: " << block_to_distance_with_v_t(st) << "!=" << block_to_distance_with_v_t(machine_state) << std::endl;
                                throw std::invalid_argument("states differ");
                            }

                            auto time1 = std::chrono::high_resolution_clock::now();

                            double dt = std::chrono::duration<double, std::milli>(time1 - time0).count();
                            std::cout << "calculations took " << dt << " milliseconds; have " << m_commands.size() << " steps to execute" << std::endl;
                            try {
                                stepping->exec(m_commands, [motor_layout_, &spindles_status, timer_drv, spindles_drv, &break_execution_result, machine_state_prev, last_spindle_on_delay](auto steps_from_origin, auto tick_n) -> int {
                                    // if (!(video->active)) {
                                    //     return 0; // finish
                                    // }
                                    std::cout << "break at " << tick_n << " tick" << std::endl;
                                    steps_from_origin = steps_from_origin + motor_layout_->cartesian_to_steps(block_to_distance_t(machine_state_prev));
                                    std::cout << "Position: " << motor_layout_->steps_to_cartesian(steps_from_origin) << std::endl;
                                    for (auto e : spindles_status) {
                                        spindles_drv->spindle_pwm_power(e.first, 0);
                                    }
                                    while (break_execution_result < 0) {
                                        timer_drv->wait_us(10000);
                                    }
                                    if ((int)(break_execution_result) == 1) {
                                        for (auto e : spindles_status) {
                                            spindles_drv->spindle_pwm_power(e.first, e.second);
                                            using namespace std::chrono_literals;
                                            std::cout << "wait for spindle..." << std::endl;
                                            std::this_thread::sleep_for(std::chrono::milliseconds(last_spindle_on_delay));
                                            std::cout << "wait for spindle... OK" << std::endl;
                                        }
                                    }
                                    int r = break_execution_result;
                                    break_execution_result = -1;
                                    return r;
                                });
                            } catch (...) {
                                spindles_drv->spindle_pwm_power(0, 0.0);
                                steppers_drv->enable_steppers({false});
                                std::cout << "TERMINATED" << std::endl;

                                return -2;
                            }
                            break;
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
                            switch ((int)(m['M'])) {
                            case 17:
                                steppers_drv->enable_steppers({true});
                                wait_for_component_to_start(m, 200);
                                break;
                            case 18:
                                steppers_drv->enable_steppers({false});
                                wait_for_component_to_start(m, 200);
                                break;
                            case 3:
                                spindles_status[0] = 1.0;
                                spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                                last_spindle_on_delay = wait_for_component_to_start(m, 3000);
                                break;
                            case 5:
                                spindles_status[0] = 0.0;
                                spindles_drv->spindle_pwm_power(0, spindles_status[0]);
                                wait_for_component_to_start(m, 3000);
                                break;
                            }
                        }
                    }
                }
                /*if (video.get() != nullptr) {
                    if (!(video->active)) {
                        stepping->terminate();
                        break;
                    }
                } */
                std::string s = "";
                for (auto e : machine_state) {
                    s = s + ((s.size()) ? " " : "") + e.first + std::to_string(e.second);
                }
                std::cout << s << std::endl;
            }
            std::cout << "FINISHED" << std::endl;
        }
    }

    return 0;
}
