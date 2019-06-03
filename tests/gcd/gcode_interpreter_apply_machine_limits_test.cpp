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
#include <chrono>
#include <gcd/gcode_interpreter.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/stepping.hpp>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;
//apply_limits_for_turns
TEST_CASE("gcode_interpreter_test - apply_limits_for_turns", "[gcd][gcode_interpreter][apply_limits_for_turns]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    program_t empty_program = {};

    program_t g1x10y20f100_program = {
        {{'G', 1},
            {'X', 10},
            {'Y', -20},
            {'F', 100}}};

    program_t g1x10y20f1_program = {
        {{'G', 1},
            {'X', 10},
            {'Y', -20},
            {'F', 1}}};

    auto x_axis_move_program = gcode_to_maps_of_arguments("G1X0Y0Z0A0F100\nG1X10Y0Z0A0F100\n");
    auto y_axis_move_program = gcode_to_maps_of_arguments("G1X0Y0Z0A0F100\nG1X0Y10Z0A0F100\n");
    auto z_axis_move_program = gcode_to_maps_of_arguments("G1X0Y0Z0A0F100\nG1X0Y0Z10A0F100\n");
    auto a_axis_move_program = gcode_to_maps_of_arguments("G1X0Y0Z0A0F100\nG1X0Y0Z0A10F100\n");
    auto turn_0_x_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F100
        G1X10Y0Z0A0F100
        G1X0Y0Z0A0F100
        )");

    SECTION("empty program results in empty program")
    {
        auto ret = apply_limits_for_turns(empty_program, machine_limits);
        REQUIRE(ret.size() == 0);
    }

    SECTION("when there is only one step, then the size of result must be 1")
    {
        auto ret = apply_limits_for_turns(g1x10y20f100_program, machine_limits);
        REQUIRE(ret.size() == 1);
    }

    SECTION("when there is only one step, then the limit should be applied")
    {
        auto ret = apply_limits_for_turns(g1x10y20f100_program, machine_limits);
        double ret_feedrate = (machine_limits.max_no_accel_velocity_mm_s[0] +
                                  machine_limits.max_no_accel_velocity_mm_s[1] +
                                  machine_limits.max_no_accel_velocity_mm_s[2] +
                                  machine_limits.max_no_accel_velocity_mm_s[3]) /
                              4;
        REQUIRE(ret[0].at('F') == Approx(ret_feedrate));
    }

    SECTION("when there is only one step")
    {
        auto ret = apply_limits_for_turns(g1x10y20f1_program, machine_limits);
        REQUIRE(ret[0]['F'] == Approx(1));
    }

    SECTION("two steps along X axis - size should be the same")
    {
        auto ret = apply_limits_for_turns(x_axis_move_program, machine_limits);
        REQUIRE(ret.size() == x_axis_move_program.size());
    }
    SECTION("two steps along X axis - first step")
    {
        auto ret = apply_limits_for_turns(x_axis_move_program, machine_limits);
        REQUIRE(ret.at(0).at('F') == Approx(machine_limits.max_no_accel_velocity_mm_s[0]));
    }
    SECTION("two steps along X axis - second step")
    {
        auto ret = apply_limits_for_turns(x_axis_move_program, machine_limits);
        INFO(back_to_gcode({x_axis_move_program}));
        INFO(back_to_gcode({remove_duplicate_blocks(x_axis_move_program,{})}));
        INFO(back_to_gcode({ret}));
        REQUIRE(ret.at(1).at('F') == Approx(machine_limits.max_no_accel_velocity_mm_s[0]));
    }
    SECTION("two steps along X axis - size should be the same - 3 steps")
    {
        auto ret = apply_limits_for_turns(turn_0_x_program, machine_limits);
        REQUIRE(ret.size() == turn_0_x_program.size());
    }
    SECTION("two steps along X axis - first step - 3 steps")
    {
        auto ret = apply_limits_for_turns(turn_0_x_program, machine_limits);
        REQUIRE(ret.at(0).at('F') == Approx(machine_limits.max_no_accel_velocity_mm_s[0]));
    }
    SECTION("two steps along X axis - last step - 3 steps")
    {
        auto ret = apply_limits_for_turns(turn_0_x_program, machine_limits);
        REQUIRE(ret.at(2).at('F') == Approx(machine_limits.max_no_accel_velocity_mm_s[0]));
    }
    SECTION("two steps along Y axis")
    {
        auto ret = apply_limits_for_turns(y_axis_move_program, machine_limits);
        REQUIRE(ret[0].at('F') == Approx (machine_limits.max_no_accel_velocity_mm_s[1]));
    }
    SECTION("two steps along Z axis")
    {
        auto ret = apply_limits_for_turns(z_axis_move_program, machine_limits);
        REQUIRE(ret[0].at('F') == Approx (machine_limits.max_no_accel_velocity_mm_s[2]));
    }
    //SECTION("two steps along A axis")
    //{
    //    auto ret = apply_limits_for_turns(a_axis_move_program, machine_limits);
    //    REQUIRE(ret[0].at('F') == Approx (machine_limits.max_no_accel_velocity_mm_s[3]));
    //}
}


TEST_CASE("gcode_interpreter_test - apply_limits_for_turns for longer program", "[gcd][gcode_interpreter][apply_limits_for_turns]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    auto turn_0_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F100
        G1X10Y0Z0A0F100
        G1X0Y0Z0A0F100
        )");
    auto turn_90_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F100
        G1X10Y0Z0A0F100
        G1X10Y10Z0A0F100
        )");
    auto turn_90_shorter_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F100
        G1X10F100
        G1Y10F100
        )");
    auto turn_45_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F100
        G1X10Y0Z0A0F100
        G1X0Y10Z0A0F100
        )");
    auto turn_180_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F200
        G1X10Y0Z0A0F300
        G1X20Y00Z0A0F400
        )");
    auto turn_135_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F200
        G1X10Y0Z0A0F300
        G1X20Y-10Z0A0F400
        )");

    auto turn_90_4x_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F100
        G1X10F100
        G1Y-10F100
        G1Z-10F100
        G1Y5F100
        G1X0F100
        G1Y0F1
        G1Z0F100
        ;G1A0F100
        )");
    SECTION("turn of 0 deg (go back) along X axis. The speed should be 0.25 of the no accel velocity")
    {
        auto ret = apply_limits_for_turns(turn_0_program, machine_limits);
        partitioned_program_t pp = {ret};
        INFO(back_to_gcode(pp));
        REQUIRE(ret[1].at('F') == Approx (machine_limits.max_no_accel_velocity_mm_s[0]*0.25));
    }
    
    SECTION("turn of 90 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity")
    {
        auto ret = apply_limits_for_turns(turn_90_program, machine_limits);
        partitioned_program_t pp = {ret};
        INFO(back_to_gcode(pp));
        REQUIRE(ret[1].at('F') == 
            Approx (std::min(
                machine_limits.max_no_accel_velocity_mm_s[0],
                machine_limits.max_no_accel_velocity_mm_s[1]
                )));
    }
    SECTION("turn of 90 deg along XY axis - shorter gcode")
    {
        auto ret = apply_limits_for_turns(turn_90_shorter_program, machine_limits);
        partitioned_program_t pp = {ret};
        INFO(back_to_gcode(pp));
        REQUIRE(ret[1].at('F') == 
            Approx (std::min(
                machine_limits.max_no_accel_velocity_mm_s[0],
                machine_limits.max_no_accel_velocity_mm_s[1]
                )));
    }
    SECTION("turn of 180 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity")
    {
        auto ret = apply_limits_for_turns(turn_180_program, machine_limits);
        partitioned_program_t pp = {ret};
        INFO(back_to_gcode(pp));
        REQUIRE(ret[1].at('F') == 
            Approx (std::min(
                machine_limits.max_velocity_mm_s[0],
                machine_limits.max_velocity_mm_s[1]
                )));
    }
    SECTION("turn of 45 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity div 2")
    {
        auto ret = apply_limits_for_turns(turn_45_program, machine_limits);
        partitioned_program_t pp = {ret};
        INFO(back_to_gcode(pp));
        double target_feedrate = (std::min(
                machine_limits.max_no_accel_velocity_mm_s[0],
                machine_limits.max_no_accel_velocity_mm_s[1]
        )*0.25+std::min(
                machine_limits.max_no_accel_velocity_mm_s[0],
                machine_limits.max_no_accel_velocity_mm_s[1]
                ))*0.5;
        REQUIRE(ret[1].at('F') == 
            Approx (target_feedrate));
        REQUIRE (ret.size() == 3);
    }

    SECTION("turn of 135 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity")
    {
        partitioned_program_t p0 = {turn_135_program};
        auto ret = apply_limits_for_turns(turn_135_program, machine_limits);
        INFO(back_to_gcode(p0));
        partitioned_program_t pp = {ret};
        INFO(back_to_gcode(pp));
        double target_feedrate = (std::min(
                machine_limits.max_velocity_mm_s[0],
                machine_limits.max_velocity_mm_s[1]
                ) + std::min(
                machine_limits.max_no_accel_velocity_mm_s[0],
                machine_limits.max_no_accel_velocity_mm_s[1]
                ))/2;
        //REQUIRE(ret[1].at('F') == Approx (target_feedrate));
        REQUIRE(ret[1].at('F') <= target_feedrate);
        REQUIRE(ret[1].at('F') > std::min(
                machine_limits.max_no_accel_velocity_mm_s[0],
                machine_limits.max_no_accel_velocity_mm_s[1]
                ));
    }

    SECTION("turn of 90 deg 4x")
    {
        auto ret = apply_limits_for_turns(turn_90_4x_program, machine_limits);
        partitioned_program_t pp = {ret};
        INFO(back_to_gcode(pp));
        REQUIRE(ret.size() == 8);
        REQUIRE(ret[0].at('F') ==  Approx(2.0));
        REQUIRE(ret[1].at('F') ==  Approx(2.0));
        REQUIRE(ret[2].at('F') ==  Approx(3.0));
        REQUIRE(ret[3].at('F') ==  Approx(3.0));
        REQUIRE(ret[4].at('F') ==  Approx(2.0));
        REQUIRE(ret[5].at('F') ==  Approx(2.0));
        REQUIRE(ret[6].at('F') ==  Approx(1.0));
        REQUIRE(ret[7].at('F') ==  Approx(4.0));
        //REQUIRE(ret[8].at('F') ==  Approx(5.0));
    }
}


TEST_CASE("gcode_interpreter_test - apply_limits_for_turns for longer program without exceeding limits", "[gcd][gcode_interpreter][apply_limits_for_turns]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {150, 151, 152, 153},     // max velocity
        {22, 23, 24, 25});        // no accel velocity

    auto turn_0_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X0Y0Z0A0F5
        )");
    auto turn_90_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X10Y10Z0A0F5
        )");
    auto turn_45_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X0Y10Z0A0F5
        )");
    auto turn_180_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X20Y00Z0A0F5
        )");
    auto turn_135_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X20Y-10Z0A0F5
        )");
    //auto turn_90_program = gcode_to_maps_of_arguments("G1X0Y0Z0A0F100\nG1X0Y10Z0A0F100\n");
    //auto turn_180_program = gcode_to_maps_of_arguments("G1X0Y0Z0A0F100\nG1X0Y0Z10A0F100\n");
    SECTION("turn of 0 deg (go back) along X axis. The speed should be 0.25 of the no accel velocity")
    {
        auto ret = apply_limits_for_turns(turn_0_program, machine_limits);
        REQUIRE(ret[1].at('F') == Approx (5.0));
    }
    
    SECTION("turn of 90 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity")
    {
        auto ret = apply_limits_for_turns(turn_90_program, machine_limits);
        REQUIRE(ret[1].at('F') == Approx (5.0));
    }
    SECTION("turn of 180 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity")
    {
        auto ret = apply_limits_for_turns(turn_180_program, machine_limits);
        REQUIRE(ret[1].at('F') == Approx (5.0));
    }
    SECTION("turn of 45 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity div 2")
    {
        auto ret = apply_limits_for_turns(turn_45_program, machine_limits);
        REQUIRE(ret[1].at('F') == Approx (5.0));
    }

    SECTION("turn of 135 deg along XY axis. The speed should be the lower of the two x and y of the no accel velocity")
    {
        partitioned_program_t p0 = {turn_135_program};
        auto ret = apply_limits_for_turns(turn_135_program, machine_limits);
        REQUIRE(ret[1].at('F') == Approx (5.0));
    }
}


TEST_CASE("Found bugs - apply_limits_for_turns","[gcd][gcode_interpreter][apply_limits_for_turns][bugs]") {
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {150, 151, 152, 153},     // max velocity
        {1, 1, 1, 1});        // no accel velocity

    SECTION("program with duplicate blocks")
    {
    auto duplicate_node_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X0Y0Z0A0F5
        )");
        program_t ret;
        REQUIRE_NOTHROW(ret = apply_limits_for_turns(duplicate_node_program, machine_limits));
        //REQUIRE(ret[0].at('F') == Approx (1.0));
        //REQUIRE(ret[1].at('F') == Approx (5.0));
    }
}

TEST_CASE("gcode_interpreter_test - remove_duplicate_blocks", "[gcd][gcode_interpreter][remove_duplicate_blocks]") {
configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {150, 151, 152, 153},     // max velocity
        {1, 1, 1, 1});        // no accel velocity

    SECTION("program with duplicate blocks should not throw")
    {
    auto duplicate_node_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X10Y0Z0A0F5
        G1X0Y0Z0A0F5
        )");
        program_t ret;
        REQUIRE_NOTHROW(ret = remove_duplicate_blocks(duplicate_node_program, {}));
    }    
    SECTION("program with duplicate blocks should have repeated blocks removed. Sample 1")
    {
    auto duplicate_node_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        M3
        G1X10Y0Z0A0F5
        G1X0Y0Z0A0F5
        M5
        )");
        program_t ret;
        REQUIRE_NOTHROW(ret = remove_duplicate_blocks(duplicate_node_program, {}));
        INFO(back_to_gcode({ret}));
        REQUIRE(ret.size() == (duplicate_node_program.size()-1));
    }
    
    SECTION("duplicate removal should also remove unnecessary duplicate node parts")
    {
    auto duplicate_node_program = gcode_to_maps_of_arguments(R"(
        G1X0Y0Z0A0F5
        G1X10Y0Z0A0F5
        M3
        G1X10Y0Z0A0F5
        G1X0Y0Z0A0F0.1
        M5
        )");
        program_t ret;
        REQUIRE_NOTHROW(ret = remove_duplicate_blocks(duplicate_node_program, {}));
        INFO(back_to_gcode({ret}));
        REQUIRE(ret[0].size() == 2);
        REQUIRE(ret[1].size() == 2);
        REQUIRE(ret[2].size() == 1);
        REQUIRE(ret[3].size() == 3);
        REQUIRE(ret[4].size() == 1);
    }
    
}


TEST_CASE("gcode_interpreter_test - block_to_distance_t", "[gcd][gcode_interpreter][block_to_distance_t]")
{
    block_t block = {
        {'G', 1},
        {'X', 10},
        {'Y', 21},
        {'Z', 22},
        //{'A', 23},
        {'F', 100}};
    SECTION("empty block should result in vector of zero")
    {
        auto ret = block_to_distance_t({});
        REQUIRE(ret[0] == Approx(0));
        REQUIRE(ret[1] == Approx(0));
        REQUIRE(ret[2] == Approx(0));
    //    REQUIRE(ret[3] == Approx(0));
    }
    SECTION("block should be converted correctly")
    {
        auto ret = block_to_distance_t(block);
        REQUIRE(ret[0] == Approx(block['X']));
        REQUIRE(ret[1] == Approx(block['Y']));
        REQUIRE(ret[2] == Approx(block['Z']));
//        REQUIRE(ret[3] == Approx(block['A']));
    }
}

TEST_CASE("gcode_interpreter_test - blocks_to_vector_move", "[gcd][gcode_interpreter][blocks_to_vector_move]")
{
    block_t block_a = {{'G', 1}, {'X', 10}, {'Y', 21}, {'Z', 22},
     //{'A', 23}, 
     {'F', 100}};
    block_t block_b = {{'G', 1}, {'X', 110}, {'Y', 121}, {'Z', 122},
     //{'A', 123}, 
     {'F', 100}};
    SECTION("empty block should result in vector of zero")
    {
        auto ret = blocks_to_vector_move({}, {});
        REQUIRE(ret[0] == Approx(0));
        REQUIRE(ret[1] == Approx(0));
        REQUIRE(ret[2] == Approx(0));
        //REQUIRE(ret[3] == Approx(0));
    }
    SECTION("block should be converted correctly")
    {
        auto ret = blocks_to_vector_move(block_a, block_b);
        REQUIRE(ret[0] == Approx(100));
        REQUIRE(ret[1] == Approx(100));
        REQUIRE(ret[2] == Approx(100));
//        REQUIRE(ret[3] == Approx(100));
    }
}
