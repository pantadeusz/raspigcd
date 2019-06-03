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

#include "tests_helper.hpp"

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;



//apply_limits_for_turns



TEST_CASE("gcode_interpreter_test - g0_move_to_g1_sequence - simple cases", "[gcd][gcode_interpreter][g0_move_to_g1_sequence][exceptions]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    program_t empty_program = {};

    program_t g0x100y_program = {
        {{'G', 0}, {'X', 100}}};
    program_t not_g0_program_program = {
        {{'G', 1}, {'X', 100}}};
    program_t not_g02_program_program = {
        {{'G', 0}, {'X', 100}},
        {{'G', 1}, {'X', 200}}};

    SECTION("empty program results in exception")
    {
        REQUIRE_THROWS_AS(
            g0_move_to_g1_sequence(empty_program, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g0_move_to_g1_sequence(empty_program, machine_limits),
            "there must be at least one G0 code in the program!");
    }

    SECTION("the program that contains other commands than G0 should be considered invalid")
    {
        REQUIRE_THROWS_AS(
            g0_move_to_g1_sequence(not_g0_program_program, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g0_move_to_g1_sequence(not_g0_program_program, machine_limits),
            "g0 should be the only type of the commands in the program for g0_move_to_g1_sequence");
        REQUIRE_THROWS_AS(
            g0_move_to_g1_sequence(not_g02_program_program, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g0_move_to_g1_sequence(not_g02_program_program, machine_limits),
            "g0 should be the only type of the commands in the program for g0_move_to_g1_sequence");
    }
}


TEST_CASE("gcode_interpreter_test - g0_move_to_g1_sequence - short movement", "[gcd][gcode_interpreter][g0_move_to_g1_sequence]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    program_t g0_short_move = {
        {{'G', 0}, {'X', 2}}};
    program_t g0_no_move = {
        {{'G', 0}, {'X', 0}}};
    SECTION("the movement too short to reach maximum speed should not throw exception")
    {
        REQUIRE_NOTHROW(g0_move_to_g1_sequence(g0_short_move, machine_limits));
    }

    SECTION("the movement too short to reach maximum speed will result in just 2 G1 commands")
    {
        auto result = g0_move_to_g1_sequence(g0_short_move, machine_limits);
        
        auto img_before = simulate_moves_on_image(g0_short_move);
        auto img_after = simulate_moves_on_image(result);
        REQUIRE(image_difference(img_before, img_after) == 0);

        REQUIRE(result.size() == 2);
        for (auto& blk : result) {
            REQUIRE(blk['G'] == 1);
        }
    }

    SECTION("the middle point should be with reachable feedrate given machine limits")
    {
        auto result = g0_move_to_g1_sequence(g0_short_move, machine_limits);
        movement::physics::path_node_t pnA = {.p = {0, 0, 0, 0}, .v = 2};
        movement::physics::path_node_t pnMed = {.p = {1, 0, 0, 0}, .v = result[0]['F']};
        //movement::physics::path_node_t pnB = {.p = {2, 0, 0, 0}, .v = 2};
        double a_real = acceleration_between(pnA, pnMed);
        REQUIRE(a_real <= (machine_limits.max_accelerations_mm_s2[0] + 0.0001));
        REQUIRE(a_real >= 0.0);
        auto img_before = simulate_moves_on_image(g0_short_move);
        auto img_after = simulate_moves_on_image(result);
        REQUIRE(image_difference(img_before, img_after) == 0);
    }

    SECTION("the middle point should be with maximal reachable feedrate given machine limits")
    {
        auto result = g0_move_to_g1_sequence(g0_short_move, machine_limits);
        movement::physics::path_node_t pnA = {.p = {0, 0, 0, 0}, .v = 2};
        movement::physics::path_node_t pnMed = {.p = {1, 0, 0, 0}, .v = result[0]['F']};
        //movement::physics::path_node_t pnB = {.p = {2, 0, 0, 0}, .v = 2};
        double a_real = acceleration_between(pnA, pnMed);
        INFO(pnA.v);
        INFO(pnMed.v);
        //INFO(pnB.v);
        REQUIRE(a_real == Approx(machine_limits.max_accelerations_mm_s2[0]));
        auto img_before = simulate_moves_on_image(g0_short_move);
        auto img_after = simulate_moves_on_image(result);
        REQUIRE(image_difference(img_before, img_after) == 0);
    }
    SECTION("zero length movement should be left unchanged")
    {
        auto result = g0_move_to_g1_sequence(g0_no_move, machine_limits);
        REQUIRE(result.size() == 1);
    }
}


TEST_CASE("gcode_interpreter_test - g0_move_to_g1_sequence - long movement", "[gcd][gcode_interpreter][g0_move_to_g1_sequence]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    program_t g0_long_move = {
        {{'G', 0}, {'X', 200}}};
    SECTION("the movement long enough to reach maximum speed should not throw exception")
    {
        REQUIRE_NOTHROW(g0_move_to_g1_sequence(g0_long_move, machine_limits));
    }
    SECTION("the long move should result in 3 steps - acceleration, constant speed and deceleration")
    {
        auto result = g0_move_to_g1_sequence(g0_long_move, machine_limits);
        REQUIRE(result.size() == 3);
        for (auto& blk : result) {
            REQUIRE(blk['G'] == 1);
        }
    }

    SECTION("constant speed should be achieved")
    {
        auto result = g0_move_to_g1_sequence(g0_long_move, machine_limits);
        REQUIRE(result[0]['F'] == Approx(50.0));
        REQUIRE(result[1]['F'] == Approx(50.0));
    }

    auto b2d = [](const block_t& block) -> distance_t {
        auto blk = block;
        return {blk['X'], blk['Y'], blk['Z'], blk['A']};
    };
    
    // auto d2b = [](const distance_t& dist)->block_t
    // {
    //     return {
    //         {'X', dist[0]},
    //         {'Y', dist[1]},
    //         {'Z', dist[2]},
    //         {'A', dist[3]},
    //     };
    // };


    SECTION("the middle point should be with maximal reachable feedrate given machine limits")
    {
        auto result = g0_move_to_g1_sequence(g0_long_move, machine_limits);
        movement::physics::path_node_t pnA = {.p = {0,0,0,0}, .v = 2};
        movement::physics::path_node_t pnMed1 = {.p = b2d(result[0]), .v = result[0]['F']};
        movement::physics::path_node_t pnMed2 = {.p = b2d(result[1]), .v = result[1]['F']};
        movement::physics::path_node_t pnB = {.p = b2d(result[2]), .v = result[2]['F']};

        INFO(pnA.p);
        INFO(pnMed1.p);
        INFO(pnMed2.p);
        INFO(pnB.p);


        double a_real1 = acceleration_between(pnA, pnMed1);
        REQUIRE(a_real1 == Approx(machine_limits.max_accelerations_mm_s2[0]));
        double a_0 = acceleration_between(pnMed1, pnMed2);
        REQUIRE(a_0 == Approx(0));
        double a_real2 = acceleration_between(pnMed2, pnB);
        REQUIRE(a_real2 == Approx(-machine_limits.max_accelerations_mm_s2[0]));

        block_t init_stat_for_check = {{'F',machine_limits.max_no_accel_velocity_mm_s[0]}, {'X',0}, {'Y',0},{'Z',0}, {'A',0} };
        auto ls_before = last_state_after_program_execution(g0_long_move, init_stat_for_check);
        auto ls_after = last_state_after_program_execution(result, init_stat_for_check);
        // std::cout << "ls_before: " << std::endl;
        // for (auto e : ls_before) {
        //     std::cout << "   " << e.first << ":" << e.second << std::endl;
        // }
        // std::cout << "ls_after: " << std::endl;
        // for (auto e : ls_after) {
        //     std::cout << "   " << e.first << ":" << e.second << std::endl;
        // }
        REQUIRE(ls_before != ls_after);
        ls_before['G'] = 1; // this must change
        REQUIRE(ls_before == ls_after);
    }


}

TEST_CASE("gcode_interpreter_test - g0_move_to_g1_sequence - multiple G0 codes", "[gcd][gcode_interpreter][g0_move_to_g1_sequence]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    program_t g0_long_moves = {
            {{'G', 0}, {'X', 200}},
            {{'G', 0}, {'Y', 200}},
            {{'G', 0}, {'Z', 200}}
        };
    SECTION("the movement long enough to reach maximum speed should not throw exception")
    {
        REQUIRE_NOTHROW(g0_move_to_g1_sequence(g0_long_moves, machine_limits));
    }
    SECTION("the long move should result in 9 steps - 3 times: acceleration, constant speed and deceleration")
    {
        auto result = g0_move_to_g1_sequence(g0_long_moves, machine_limits);
        REQUIRE(result.size() == 9);
        for (auto& blk : result) {
            REQUIRE(blk['G'] == 1);
        }
    }

    SECTION("constant speed should be achieved 3 times")
    {
        auto result = g0_move_to_g1_sequence(g0_long_moves, machine_limits);
        int cspeed = 0;
        for (unsigned int i = 1; i < result.size();i++) {
            if ((result[i-1]['F'] == result[i]['F']) && (result[i-1]['F'] >= 50.0)) cspeed++;
        }
        REQUIRE(cspeed == 3);
    }

    SECTION("slow speed should be achieved 3 times")
    {
        auto result = g0_move_to_g1_sequence(g0_long_moves, machine_limits);
        int cspeed = 0;
        for (unsigned int i = 0; i < result.size();i++) {
            if ((result[i]['F'] >= 2) && (result[i]['F'] < 50.0)) cspeed++;
        }
        REQUIRE(cspeed == 3);
    }
 // todo: accelerations check
}
