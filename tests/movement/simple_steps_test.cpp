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


#include <configuration.hpp>
#include <configuration_json.hpp>
#include <hardware/stepping.hpp>
#include <movement/simple_steps.hpp>

#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::movement::simple_steps;
using namespace raspigcd::movement;


TEST_CASE("Movement constant speed in steps generator", "[movement][steps_generator]")
{
    hardware::single_step_command zero_move{.step = 0, .dir = 0};
    SECTION("collapse_repeated_steps")
    {
        auto result = collapse_repeated_steps({});
        REQUIRE(result.size() == 0);

        result = collapse_repeated_steps({{
            .b = {zero_move, zero_move, zero_move, zero_move},
            .count = 1}});
        REQUIRE(result.size() == 1);

        result = collapse_repeated_steps(
            {
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 1},
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 1}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 2);

        result = collapse_repeated_steps(
            {
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 1},
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 0}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 1);

        result = collapse_repeated_steps(
            {
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 0},
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 0}
            }
            );
        REQUIRE(result.size() == 0);

        result = collapse_repeated_steps(
            {
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 1},
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 2}
            }
            );
        REQUIRE(result.size() == 1);
        REQUIRE(result[0].count == 3);

        result = collapse_repeated_steps(
            {
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 1},
            {.b = {zero_move, zero_move, zero_move, zero_move}, .count = 2},
            {.b = {hardware::single_step_command{1,0}, hardware::single_step_command{1, 0}, zero_move, zero_move}, .count = 2}
            }
            );
        REQUIRE(result.size() == 2);
        REQUIRE(result[0].count == 3);
        REQUIRE(result[1].count == 2);
    }
}
