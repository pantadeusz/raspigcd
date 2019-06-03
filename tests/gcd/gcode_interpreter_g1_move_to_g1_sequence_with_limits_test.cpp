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
#include <movement/physics.hpp>
#include <thread>
#include <vector>


#include "tests_helper.hpp"

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;
//apply_limits_for_turns
TEST_CASE("gcode_interpreter_test - g1_move_to_g1_with_machine_limits - simple cases", "[gcd][gcode_interpreter][g1_move_to_g1_with_machine_limits][exceptions]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity

    program_t program = gcode_to_maps_of_arguments("");
    program_t program_not_g0_g1_a = gcode_to_maps_of_arguments("G4X20M10\nM17");
    program_t program_not_g0_g1_b = gcode_to_maps_of_arguments("M17\nM5");
    program_t empty_program = {};

    SECTION("empty program results in exception")
    {
        REQUIRE_THROWS_AS(
            g1_move_to_g1_with_machine_limits(empty_program, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g1_move_to_g1_with_machine_limits(empty_program, machine_limits),
            "there must be at least one G0 or G1 code in the program!");
    }

    SECTION("the program that contains other commands than G0 or G1 should be considered invalid")
    {
        REQUIRE_THROWS_AS(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_a, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_a, machine_limits),
            "Gx should be the only type of the commands in the program for g1_move_to_g1_with_machine_limits");
        REQUIRE_THROWS_AS(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_b, machine_limits),
            std::invalid_argument);
        REQUIRE_THROWS_WITH(
            g1_move_to_g1_with_machine_limits(program_not_g0_g1_b, machine_limits),
            "Gx should be the only type of the commands in the program for g1_move_to_g1_with_machine_limits");
    }
}

TEST_CASE("gcode_interpreter_test - g1_move_to_g1_with_machine_limits - check if resulting gcode contains correct path", "[gcd][gcode_interpreter][g1_move_to_g1_with_machine_limits][in_path]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {50, 51, 52, 53},     // max velocity
        {2, 3, 4, 5});        // no accel velocity
    program_t program_0 = gcode_to_maps_of_arguments("G1X10F1");

    SECTION("program without any acceleration that must be reduced")
    {
        auto program_0_prim = g1_move_to_g1_with_machine_limits(program_0, machine_limits);
        INFO(back_to_gcode({program_0}));
        INFO(back_to_gcode({program_0_prim}));
        auto img_before = simulate_moves_on_image(program_0);
        auto img_after = simulate_moves_on_image(program_0_prim);
        REQUIRE(image_difference(img_before, img_after) == 0);
    }
}

TEST_CASE("gcode_interpreter_test - g1_move_to_g1_with_machine_limits - check if machine limits are not breached", "[gcd][gcode_interpreter][g1_move_to_g1_with_machine_limits][in_limits]")
{
    using namespace raspigcd::movement::physics;
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {20, 21, 22, 23},     // max velocity
        {2, 3, 4, 5});        // no accel velocity
 
    SECTION("sharp turns should be performed with lower speed")
    {
        program_t program_0 = gcode_to_maps_of_arguments("G1X10F1\nG1Y20F50\nG1X0Z10F30\nG1X0Y0Z0F1");
        auto program_0_prim = g1_move_to_g1_with_machine_limits(program_0, machine_limits);
        INFO(back_to_gcode({program_0}));
        INFO(back_to_gcode({program_0_prim}));
        REQUIRE(program_0_prim.size() == 4);
        REQUIRE(program_0_prim.at(0).at('F') == Approx(1.0));
        REQUIRE(program_0_prim.at(1).at('F') < 21.0);
        REQUIRE(program_0_prim.at(1).at('F') == Approx(3.0));
        REQUIRE(program_0_prim.at(2).at('F') < 22.0);
    }

    // double acceleration_between(const path_node_t &a, const path_node_t &b) {
        
    SECTION("accelerations should fit in machine limits") {
        program_t program_0 = gcode_to_maps_of_arguments("G1X1F1\nG1X1.5F20\nG1X2F1");
        auto program_0_prim = g1_move_to_g1_with_machine_limits(program_0, machine_limits);
        INFO(back_to_gcode({program_0}));
        INFO(back_to_gcode({program_0_prim}));
        for (unsigned i = 1; i < program_0_prim.size(); i++) {
//            REQUIRE(program_0_prim.at(0).at('F') == Approx(1.0));
            path_node_t p0n = {
                .p = block_to_distance_t(program_0_prim.at(i-1)),
                .v = program_0_prim.at(i-1)['F']};
            path_node_t p1n = {
                .p = block_to_distance_t(program_0_prim.at(i)),
                .v = program_0_prim.at(i)['F']};
            
            double accel = acceleration_between(p0n, p1n);
            REQUIRE(accel < 103);
            REQUIRE(accel > -103);
        }
    }
    
    // double acceleration_between(const path_node_t &a, const path_node_t &b) {
    SECTION("sharp brake should also be well resolved") {
        program_t program_0 = gcode_to_maps_of_arguments("G1X1F1\nG1X11.5F20\nG1X12F1");
        auto program_0_prim = g1_move_to_g1_with_machine_limits(program_0, machine_limits);
        INFO(back_to_gcode({program_0}));
        INFO(back_to_gcode({program_0_prim}));
        for (unsigned i = 1; i < program_0_prim.size(); i++) {
//            REQUIRE(program_0_prim.at(0).at('F') == Approx(1.0));
            path_node_t p0n = {
                .p = block_to_distance_t(program_0_prim.at(i-1)),
                .v = program_0_prim.at(i-1)['F']};
            path_node_t p1n = {
                .p = block_to_distance_t(program_0_prim.at(i)),
                .v = program_0_prim.at(i)['F']};
            
            double accel = acceleration_between(p0n, p1n);
            REQUIRE(accel < 103);
            REQUIRE(accel > -103);
        
        }
    } 
    // double acceleration_between(const path_node_t &a, const path_node_t &b) {
    SECTION("breaking cannot exceed speed limits") {
        program_t program_0 = gcode_to_maps_of_arguments("G1X1F1\nG1X11.5F20\nG1X12F1");
        auto program_0_prim = g1_move_to_g1_with_machine_limits(program_0, machine_limits);
        INFO(back_to_gcode({program_0}));
        INFO(back_to_gcode({program_0_prim}));
        REQUIRE(program_0_prim.at(0)['F'] <= 1.0);
        REQUIRE(program_0_prim.at(0)['F'] >= -1.0);
        REQUIRE(program_0_prim.at(1)['F'] <=  20.0);
        REQUIRE(program_0_prim.at(1)['F'] >= -20.0);
        REQUIRE(program_0_prim.at(2)['F'] <=  1.0);
        REQUIRE(program_0_prim.at(2)['F'] >= -1.0);
    }
}

//TEST_CASE("gcode_interpreter_test - g1_move_to_g1_with_machine_limits - check if resulting gcode is within machine limits", "[gcd][gcode_interpreter][g1_move_to_g1_with_machine_limits][in_limits]")
//{
//
//}
