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

TEST_CASE("gcode_interpreter_test - insert_additional_nodes_inbetween", "[gcd][gcode_interpreter][insert_additional_nodes_inbetween]")
{
    configuration::limits machine_limits(
        {100, 101, 102, 103}, // acceleration
        {150, 151, 152, 153},     // max velocity
        {22, 23, 24, 25});        // no accel velocity
    SECTION("it should compile")
    {
        REQUIRE(insert_additional_nodes_inbetween != nullptr);
    }

    SECTION("it should not throw in case of empty parameters")
    {
        partitioned_program_t result;
        partitioned_program_t input = {};
        block_t initial_state = {};
        REQUIRE_NOTHROW(result = insert_additional_nodes_inbetween(input, initial_state, machine_limits));
    }
    
    SECTION("empty parameters should result in empty result")
    {
        partitioned_program_t result;
        partitioned_program_t input = {};
        block_t initial_state = {};
        result = insert_additional_nodes_inbetween(input, initial_state, machine_limits);
        REQUIRE(result.size() == 0);
    }

    SECTION("program without any G moves should result in the unmodified one")
    {
        partitioned_program_t result;
        block_t initial_state = {};
        auto program = gcode_to_maps_of_arguments("M17\nM18\nM5\nM3");
        auto input = group_gcode_commands(program);

        result = insert_additional_nodes_inbetween(input, initial_state, machine_limits);
        REQUIRE(result.size() == input.size());
        for (unsigned i = 0; i < result.size(); i++) {
            REQUIRE(result[i] == input[i]);
        }
    }
    SECTION("program without G code move that goes nowhere should result in the unmodified one")
    {
        partitioned_program_t result;
        block_t initial_state = {};
        auto program = gcode_to_maps_of_arguments("M17\nM18\nG0X0\nM5\nM3");
        auto input = group_gcode_commands(program);

        result = insert_additional_nodes_inbetween(input, initial_state, machine_limits);
        REQUIRE(result.size() == input.size());
        for (unsigned i = 0; i < result.size(); i++) {
            REQUIRE(result[i] == input[i]);
        }
    }
   
    SECTION("program with G codes that moves in short distance should give additional 1 step")
    {
        partitioned_program_t result;
        block_t initial_state = {};
        auto program = gcode_to_maps_of_arguments("M17\nM18\nG0X1F100\nM5\nM3");
        auto input = group_gcode_commands(program);

        result = insert_additional_nodes_inbetween(input, initial_state, machine_limits);
        REQUIRE(result.size() == input.size());
        for (unsigned i = 0; i < result.size(); i++) {
            if (i == 2) {
                INFO(i);
                INFO(result[i].size());
                REQUIRE(result[i].size() == (input[i].size()+1));
                REQUIRE(result[i][0]['G'] == input[i][0]['G']);
                REQUIRE(result[i][1]['G'] == input[i][0]['G']);
                REQUIRE(result[i][0]['F'] == input[i][0]['F']);
                REQUIRE(result[i][1]['F'] == input[i][0]['F']);
            } else {
                REQUIRE(result[i].size() == input[i].size());
            }
        }
    }

    SECTION("program with G codes that moves in long distance should give additional 2 steps")
    {
        partitioned_program_t result;
        block_t initial_state = {};
        auto program = gcode_to_maps_of_arguments("M17\nM18\nG0X100F100\nM5\nM3");
        auto input = group_gcode_commands(program);

        result = insert_additional_nodes_inbetween(input, initial_state, machine_limits);
        REQUIRE(result.size() == input.size());
        for (unsigned i = 0; i < result.size(); i++) {
            if (i == 2) {
                INFO(i);
                INFO(result[i].size());
                REQUIRE(result[i].size() == (input[i].size()+2));
                REQUIRE(result[i][0]['G'] == input[i][0]['G']);
                REQUIRE(result[i][1]['G'] == input[i][0]['G']);
                REQUIRE(result[i][2]['G'] == input[i][0]['G']);
                REQUIRE(result[i][0]['F'] == input[i][0]['F']);
                REQUIRE(result[i][1]['F'] == input[i][0]['F']);
                REQUIRE(result[i][2]['F'] == input[i][0]['F']);
            } else {
                REQUIRE(result[i].size() == input[i].size());
            }
        }
    }
}