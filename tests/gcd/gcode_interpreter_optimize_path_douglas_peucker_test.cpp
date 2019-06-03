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

#include <catch2/catch.hpp>
#include <chrono>
#include <gcd/gcode_interpreter.hpp>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::gcd;

TEST_CASE("gcode_interpreter_test - optimize_path_douglas_peucker", "[gcd][gcode_interpreter][optimize_path_douglas_peucker]")
{

    SECTION("empty gives empty")
    {
        REQUIRE_NOTHROW (optimize_path_douglas_peucker({},0.0125));
    }
    SECTION("only m-codes gives the same result")
    {
        program_t input = {
            {{'M',3}},
            {{'M',17}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        REQUIRE(result == input);
    }
    SECTION("One move gives the same result")
    {
        program_t input = {
            {{'G',0},{'X',17}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        REQUIRE(result == input);
    }
    SECTION("One move with double element gives the shorter result")
    {
        program_t input = {
            {{'G',0},{'X',17}},
            {{'G',0},{'X',17}}
        };
        program_t expected = {
            {{'G',0},{'X',17}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        REQUIRE(result == expected);
    }
    SECTION("One move with same place but different velocities gives the same result")
    {
        program_t input = {
            {{'G',0},{'X',17},{'F',10}},
            {{'G',0},{'X',17},{'F',20}}
        };
        program_t expected = {
            {{'G',0},{'X',17},{'F',10}},
            {{'G',0},{'X',17},{'F',20}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        INFO(back_to_gcode({input}));
        INFO(back_to_gcode({result}));
        REQUIRE(result == expected);
    }

    SECTION("G4 must be treated with respect")
    {
        program_t input = {
            {{'G',4},{'X',17}},
            {{'G',0},{'X',18}}
        };
        program_t expected = {
            {{'G',4},{'X',17}},
            {{'G',0},{'X',18}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        REQUIRE(result == expected);
    }
    SECTION("G4 and then move to the new position should not be removed")
    {
        program_t input = {
            {{'G',4},{'X',17}},
            {{'G',0},{'X',17}}
        };
        program_t expected = {
            {{'G',4},{'X',17}},
            {{'G',0},{'X',17}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        REQUIRE(result == expected);
    }
    SECTION("mixed G and M codes should be interpreted correctly")
    {
        program_t input = {
            {{'M',5}},
            {{'G',0},{'X',5}},
            {{'M',4}},
            {{'G',0},{'X',17}}
        };
        program_t expected = {
            {{'M',5}},
            {{'G',0},{'X',5}},
            {{'M',4}},
            {{'G',0},{'X',17}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        INFO(back_to_gcode({result}));
        INFO(back_to_gcode({expected}));

        REQUIRE(result == expected);
    }
    SECTION("mixed G and M codes should be interpreted correctly with removal of correct node")
    {
        program_t input = {
            {{'M',5}},
            {{'G',0},{'X',5}},
            {{'M',4}},
            {{'G',0},{'X',5}}
        };
        program_t expected = {
            {{'M',5}},
            {{'G',0},{'X',5}},
            {{'M',4}},
            {{'G',0},{'X',5}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        REQUIRE(result == expected);
    }
    SECTION("mixed G and M codes should be interpreted correctly with removal of correct node 2")
    {
        program_t input = {
            {{'M',5}},
            {{'G',0},{'X',5}},
            {{'G',0},{'X',5}},
            {{'M',4}},
            {{'G',0},{'X',5}}
        };
        program_t expected = {
            {{'M',5}},
            {{'G',0},{'X',5}},
            {{'M',4}},
            {{'G',0},{'X',5}}
        };
        auto result =  optimize_path_douglas_peucker(input,0.0125);
        REQUIRE(result == expected);
    }
    
    }
