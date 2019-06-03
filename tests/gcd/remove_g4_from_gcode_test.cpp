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
#include <gcd/remove_g92_from_gcode.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware/stepping.hpp>
#include <gcd/gcode_interpreter.hpp>
#include <gcd/gcode_interpreter.hpp>

#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::gcd;

TEST_CASE("converters - remove_g92_from_gcode", "[gcd][converters][remove_g92_from_gcode]")
{
    //program_t program = gcode_to_maps_of_arguments("");

    SECTION("the function should compile - remove_g92_from_gcode") {
        const program_t empty_program = {};
        program_t empty2;
        REQUIRE_NOTHROW(empty2 = remove_g92_from_gcode(empty_program));
    }
    
    SECTION("empty gcode should give empty result") {
        const program_t empty_program = {};
        REQUIRE(remove_g92_from_gcode(empty_program).size() == 0);
    }
}


TEST_CASE("converters - remove_g92_from_gcode - gcodes withould G92", "[gcd][converters][remove_g92_from_gcode][no_change]")
{
    SECTION("Test case 1") {
        program_t program = gcode_to_maps_of_arguments("G0X10\nG1X5Y5\nG0X0Y0\n");
        CHECK(program.size() == 3);
        REQUIRE(remove_g92_from_gcode(program).size() == 3);
        REQUIRE(remove_g92_from_gcode(program) == program);
    }
    SECTION("Test case 2") {
        program_t program = gcode_to_maps_of_arguments("G0X10\nM17\nG1X5Y5\nG0X0Y0\nM3");
        CHECK(program.size() == 5);
        REQUIRE(remove_g92_from_gcode(program).size() == 5);
        REQUIRE(remove_g92_from_gcode(program) == program);
    }
    SECTION("Test case 3") {
        program_t program = gcode_to_maps_of_arguments("G0X10\n");
        CHECK(program.size() == 1);
        REQUIRE(remove_g92_from_gcode(program).size() == 1);
        REQUIRE(remove_g92_from_gcode(program) == program);
    }
    SECTION("Test case 4") {
        program_t program = gcode_to_maps_of_arguments("G0X10F20\nG1X5Y5\nG0X0Y0\n");
        CHECK(program.size() == 3);
        REQUIRE(remove_g92_from_gcode(program).size() == 3);
        REQUIRE(remove_g92_from_gcode(program) == program);
    }
}

TEST_CASE("converters - remove_g92_from_gcode - gcodes with G92", "[gcd][converters][remove_g92_from_gcode][G92moves]")
{
    SECTION("Test case 1 - we should remove G92 from gcodes") {
        program_t program = gcode_to_maps_of_arguments("G0X10\nG92X10\nG0X-10");

        REQUIRE(remove_g92_from_gcode(program).size() == 2);
    }
    SECTION("Test case 2 - we should go back to 0 for the gcode that goes G0X10 G92X10 G0X-10") {
        program_t program = gcode_to_maps_of_arguments("G0X10\nG92X10\nG0X0");

        auto result = remove_g92_from_gcode(program);
        REQUIRE(result.at(0).at('X') == Approx(10.0));
        REQUIRE(result.at(1).at('X') == Approx(0.0));
    }
    SECTION("Test case 3 - we should go back to 0 for the gcode that goes G0X10 G92Y10 G0X0Y10") {
        program_t program = gcode_to_maps_of_arguments("G0X10\nG92Y10\nG0X0Y10");

        auto result = remove_g92_from_gcode(program);
        REQUIRE(result.at(0).at('X') == Approx(10.0));
        REQUIRE(result.at(1).at('X') == Approx(0.0));
        REQUIRE(result.at(1).at('Y') == Approx(0.0));
    }
}