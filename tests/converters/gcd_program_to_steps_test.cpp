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


#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <converters/gcd_program_to_steps.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware/stepping.hpp>
#include <gcd/gcode_interpreter.hpp>
#include "tests_helper.hpp"

#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::gcd;
using namespace raspigcd::converters;

// steps_t steps_size_maker;
const unsigned COORDINATES_COUNT = 3;

TEST_CASE("converters - program_to_steps", "[gcd][converters][program_to_steps]")
{
    configuration::actuators_organization test_config;

    test_config.motion_layout = configuration::motion_layouts::CARTESIAN;
    test_config.scale = {1,1,1,1};
    test_config.tick_duration_us = 100; // 0.0001 s

    for (size_t i = 0; i <COORDINATES_COUNT;i++){
        // auto & stepper: test_config.steppers) {
        configuration::stepper stepper;
        stepper.dir = 1;
        stepper.en = 2;
        stepper.step = 3;
        stepper.steps_per_mm = 100;
        test_config.steppers.push_back(stepper);
    }
    steps_t steps_per_mm_arr;
    for (size_t i = 0; i < COORDINATES_COUNT; i++) {
        steps_per_mm_arr[i] = test_config.steppers[i].steps_per_mm;
    };

    auto motor_layot_p = hardware::motor_layout::get_instance(test_config);
    motor_layot_p->set_configuration(test_config);
    //std::cout << "sdfdss::"<<(motor_layot_p.get()->cartesian_to_steps({1,2,3,4})) << "   <<<)))))_____  \n";
    auto program_to_steps = converters::program_to_steps_factory("program_to_steps");

    SECTION("empty program should result in empty steps list")
    {
        REQUIRE(program_to_steps(
            {},test_config, *(motor_layot_p.get()) 
            ,{{'F',0}},[](const gcd::block_t &){}
            ) .size() == 0);
    }
    SECTION("simple move along x axis for 100 steps should result in correct distance generated")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1X1F10
        )");
        auto result = program_to_steps(
            program,test_config, *(motor_layot_p.get()),{{'F',0}},[](const gcd::block_t &){} );
        //REQUIRE(result.size() == 200); // empty+step * 100
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (size_t i = 0; i < COORDINATES_COUNT; i++) {
                    auto m = e.b[i];
                    //std::cout << "s:" << m.step << " " << m.dir;
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
                //std::cout << std::endl;
            }
        }
        REQUIRE(steps == steps_t{100,0,0,0});
    }
    SECTION("simple move along x axis for 100 steps should result in correct speed")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F1
           G1X1F1
        )");
        // 1mm/s, 
        // 1s is  1000000/test_config.tick_duration_us -> this is the time of the movement
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) ,{{'F',0}},[](const gcd::block_t &){});
        //REQUIRE(result.size() == 200); // empty+step * 100
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (size_t i = 0; i < COORDINATES_COUNT; i++) {
                    auto m = e.b[i];
                    //std::cout << "s:" << m.step << " " << m.dir;
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
                //std::cout << std::endl;
            }
        }
        REQUIRE(steps == steps_t{100,0,0,0});
        REQUIRE(commands_count == (1000000/test_config.tick_duration_us));
    }
    SECTION("if the speed is 0 and the distance is not 0, then the exception should be throwned")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F0
        )");
        REQUIRE_THROWS( program_to_steps(program,test_config, *(motor_layot_p.get()),{{'F',0}},[](const gcd::block_t &){} ));
    }
    SECTION("acceleration from F0 to F1 should result in correct distance")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F1
        )");
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) ,{{'F',0}},[](const gcd::block_t &){});
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (size_t i = 0; i < COORDINATES_COUNT; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(steps == steps_t{100,0,0,0});
    }

    SECTION("acceleration from F0 to F1 should result in correct time")
    {
        double t = 1;
        double a = 100;
        double s = a*t*t/2.0;
        double v1 = a*t;
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           )" +
           std::string("G1X") + std::to_string(s) + "F" + std::to_string(v1)
        );
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) ,{{'F',0}},[](const gcd::block_t &){});
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (size_t i = 0; i < COORDINATES_COUNT; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(steps == steps_t{(int)(s*100),0,0,0});
        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(commands_count == (int)(t/dt));
//        REQUIRE(result.size() == (1000000/test_config.tick_duration_us));
    }
    SECTION("break from F1 to F0 should result in correct time")
    {
        double t = 1;
        double a = -10;
        double v0 = 10;
        double v1 = v0 + a*t;
        v1 = 0;
        double s = v0 + a*t*t/2.0;
        auto program = gcode_to_maps_of_arguments(
           std::string("G1X") + std::to_string(0) + "F" + std::to_string(v0) + "\n" +
           std::string("G1X") + std::to_string(s) + "F" + std::to_string(v1)
        );
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()) ,{{'F',0}},[](const gcd::block_t &){});
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (size_t i = 0; i < COORDINATES_COUNT; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(steps == steps_t{(int)(s*100.0),0,0,0});
        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(commands_count == (int)(t/dt));
    }
    SECTION("program_to_steps should optimize the commands count - reduce it")
    {
        double t = 1;
        double a = -10;
        double v0 = 10;
        double v1 = v0 + a*t;
        v1 = 0;
        double s = v0 + a*t*t/2.0;
        auto program = gcode_to_maps_of_arguments(
           std::string("G1X") + std::to_string(0) + "F" + std::to_string(v0) + "\n" +
           std::string("G1X") + std::to_string(s) + "F" + std::to_string(v1)
        );
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),{{'F',0}} ,[](const gcd::block_t &){});
        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (size_t i = 0; i < COORDINATES_COUNT; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }
        REQUIRE(commands_count > result.size() );
    }
    
    SECTION("end state verification")
    {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F1
           G1X10Y20Z10F4
           G1X2Y2Z3A4
           G1X1
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );

        steps_t steps = {0,0,0,0};
        int commands_count = 0; 
        for (auto &e : result) {
            for (int i = 0; i < e.count; i++) {
                commands_count++;
                for (size_t i = 0; i < COORDINATES_COUNT; i++) {
                    auto m = e.b[i];
                    if (m.step) steps[i] += ((int)(m.dir)*2)-1;
                }
            }
        }

        steps_t expected_steps = steps_per_mm_arr*steps_t{1,2,3,4};
        REQUIRE(steps == expected_steps);
        std::list<steps_t> hstps = hardware_commands_to_steps(result);
        REQUIRE(hstps.back() == expected_steps);

        //double dt = ((double) test_config.tick_duration_us)/1000000.0;
        //REQUIRE(commands_count == (int)(t/dt));


        REQUIRE(machine_state['F'] == Approx(4));
        REQUIRE(machine_state['X'] == Approx(1));
        REQUIRE(machine_state['Y'] == Approx(2));
        REQUIRE(machine_state['Z'] == Approx(3));
        REQUIRE(machine_state['A'] == Approx(4));
        auto ls = last_state_after_program_execution(program, {{'F',1}});
        REQUIRE(ls == machine_state);
    }

    SECTION("accept coordinates change via G92") {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F0
           G1X1F1
           G1X10Y20Z10F4
           G1X0Y0Z0F4
           G92X101Y102Z103A104
           G1X2Y2Z3A4
           G1X1
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );
        REQUIRE(machine_state['F'] == Approx(4));
        REQUIRE(machine_state['X'] == Approx(1));
        REQUIRE(machine_state['Y'] == Approx(2));
        REQUIRE(machine_state['Z'] == Approx(3));
        REQUIRE(machine_state['A'] == Approx(4));
        auto ls = last_state_after_program_execution(program, {{'F',1}});
        REQUIRE(ls == machine_state);

        steps_t expected_steps = steps_per_mm_arr*steps_t{-100,-100,-100};
        std::list<steps_t> hstps = hardware_commands_to_steps(result);
        REQUIRE(hstps.back() == expected_steps);
/*
        std::vector<std::vector<int>> img_dta = simulate_moves_on_image(program);
        std::vector<std::vector<int>> img_dta_2 = simulate_moves_on_image(result,*(motor_layot_p.get()));
        
    //int image_difference(const std::vector<std::vector<int>>& a, const std::vector<std::vector<int>>& b);



    //std::vector<std::vector<int>> load_image(std::string filename);
        save_image(std::string(__FILE__)+"_accept_ccordinates_change", img_dta);
        */
    } 

    SECTION("simple dwell G04 for 1 second - position is correct") {
        auto program = gcode_to_maps_of_arguments(R"(
           G4X1
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );
        auto ls = last_state_after_program_execution(program, {{'X',0},{'F',1}});
        REQUIRE(ls['X'] == machine_state['X']);
        REQUIRE(ls['Y'] == machine_state['Y']);
        REQUIRE(ls['Z'] == machine_state['Z']);
        REQUIRE(ls['A'] == machine_state['A']);

        steps_t expected_steps = steps_per_mm_arr*steps_t{0,0,0,0};
        steps_t hstps = hardware_commands_to_last_position_after_given_steps(result);
        REQUIRE(hstps == expected_steps);
    } 
    
    SECTION("simple dwell G04 for 1 second - time is correct") {
        auto program = gcode_to_maps_of_arguments(R"(
           G4X1
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );
        auto ls = last_state_after_program_execution(program, {{'F',1}});
        int steps_done_count = hardware_commands_to_steps_count(result);

        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(steps_done_count == (int)(1.0/dt));
    }

    SECTION("simple dwell G04 for 2 second in miliseconds - time is correct") {
        auto program = gcode_to_maps_of_arguments(R"(
           G4P2000
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );
        auto ls = last_state_after_program_execution(program, {{'F',1}});
        int steps_done_count = hardware_commands_to_steps_count(result);

        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(steps_done_count == (int)(2.0/dt));
    }

    SECTION("simple dwell G04 for 2 second in miliseconds with additional G1 commands that do nothing- time is correct") {
        auto program = gcode_to_maps_of_arguments(R"(
           G1F1
           G4P1500
           G1F1
           G4P500
        )");
        block_t machine_state = {{'F',1}};
        auto result = program_to_steps(program,test_config, *(motor_layot_p.get()),
        machine_state, [&machine_state](const block_t &result){
            machine_state = result;
        } );
        auto ls = last_state_after_program_execution(program, {{'F',1}});
        //INFO(result);
        int steps_done_count = hardware_commands_to_steps_count(result);
        INFO(steps_done_count);
        double dt = ((double) test_config.tick_duration_us)/1000000.0;
        REQUIRE(steps_done_count == (int)(2.0/dt));
    }

}
