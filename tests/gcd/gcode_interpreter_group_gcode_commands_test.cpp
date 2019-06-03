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

TEST_CASE("gcode_interpreter_test - group_gcode_commands", "[gcd][gcode_interpreter][group_gcode_commands]")
{
    auto simple_gcode_one_group_g1 = gcode_to_maps_of_arguments(R"(
        G1X10
        G1Y10
        G1X0
        G1Y0
    )");
    auto simple_gcode_seven_groups_mg1g0g1m = gcode_to_maps_of_arguments(R"(
        M17
        M3
        G1Z5
        G1X10
        G1Y10
        G1X0
        G1Y0
        G0X10Y10
        G1X20Y30
        G1X0
        G1Y0
        G1Z0
        M5
        M18
    )");
    auto simple_gcode_one_group_g1_w_error = gcode_to_maps_of_arguments(R"(
        Y10
        G1X10
        G1X0
        G1Y0
    )");
    auto simple_gcode_one_group_g1m_w_error = gcode_to_maps_of_arguments(R"(
        G0X10
        M5
        Y10
        G1X10
        G1X0
        G1Y0
    )");
    auto simple_gcode_one_group_g1_wo_error = gcode_to_maps_of_arguments(R"(
        G0X10
        Y10
        G1X10
        G1X0
        G1Y0
    )");    
    auto simple_gcode_one_group_g1_with_feedrate_set = gcode_to_maps_of_arguments(R"(
        G1X10F10
        G1Y10
        G1X0
        G1Y0
    )");

    auto simple_gcode_multiple_groups_feedrate_shold_be_recorded_correctly = gcode_to_maps_of_arguments(R"(
        G1X10F10
        G1Y10
        M5
        G1X0 ; we expect here to have F10
        G1Y0
    )");


    SECTION("empty gives empty")
    {
        auto ret = group_gcode_commands({});
        REQUIRE(ret.size() == 0);
    }
    SECTION("Single group simple_gcode_one_group_g1")
    {
        auto ret = group_gcode_commands(simple_gcode_one_group_g1);
        REQUIRE(ret.size() == 1);
    }
    SECTION("simple_gcode_seven_groups_mg1g0g1m shoulod give 7 groups")
    {
        auto ret = group_gcode_commands(simple_gcode_seven_groups_mg1g0g1m);
        REQUIRE(ret.size() == 7);
    }
    SECTION("simple_gcode_seven_groups_mg1g0g1m groups should not be mixed")
    {
        auto ret_a = group_gcode_commands(simple_gcode_seven_groups_mg1g0g1m);
        auto ret = std::vector<program_t>(ret_a.begin(), ret_a.end());
        INFO(back_to_gcode(ret_a));
        REQUIRE(ret[0].size() == 1);
        REQUIRE(ret[1].size() == 1);
        REQUIRE(ret[2].size() == 6);
        REQUIRE(ret[3].size() == 1);
        REQUIRE(ret[4].size() == 5);
        REQUIRE(ret[5].size() == 1);
        REQUIRE(ret[6].size() == 1);
    }
    // simple_gcode_seven_groups_mg1g0g1m

    SECTION("Single group simple_gcode_one_group_g1 should result in additional command at start")
    {
        auto ret = group_gcode_commands(simple_gcode_one_group_g1,{{'F',3}});
        //INFO(back_to_gcode(ret));
        REQUIRE(ret.size() == 1);
        REQUIRE(ret.front().size() == 5);
        REQUIRE(ret.front().at(0).at('F') == Approx(3));  REQUIRE(ret.front().at(0).at('G') == Approx(1));
        REQUIRE(ret.front().at(1).at('G') == Approx(1));  REQUIRE(ret.front().at(1).at('X') == Approx(10));
        REQUIRE(ret.front().at(2).at('G') == Approx(1));  REQUIRE(ret.front().at(2).at('Y') == Approx(10));
        REQUIRE(ret.front().at(3).at('G') == Approx(1));  REQUIRE(ret.front().at(3).at('X') == Approx(0));
        REQUIRE(ret.front().at(4).at('G') == Approx(1));  REQUIRE(ret.front().at(4).at('Y') == Approx(0));
    }

    SECTION("Single group simple_gcode_one_group_g1_with_feedrate_set should have correct F set")
    {
        auto ret = group_gcode_commands(simple_gcode_one_group_g1_with_feedrate_set,{{'F',3}});
        //INFO(back_to_gcode(ret));
        REQUIRE(ret.size() == 1);
        REQUIRE(ret.front().size() == 4);
        REQUIRE(ret.front().at(0).at('F') == Approx(10));  REQUIRE(ret.front().at(0).at('G') == Approx(1)); REQUIRE(ret.front().at(0).at('X') == Approx(10));
        REQUIRE(ret.front().at(1).at('G') == Approx(1));  REQUIRE(ret.front().at(1).at('Y') == Approx(10));
        REQUIRE(ret.front().at(2).at('G') == Approx(1));  REQUIRE(ret.front().at(2).at('X') == Approx(0));
        REQUIRE(ret.front().at(3).at('G') == Approx(1));  REQUIRE(ret.front().at(3).at('Y') == Approx(0));
    }

    SECTION("Multiple group simple_gcode_multiple_groups_feedrate_shold_be_recorded_correctly should have correct F set")
    {
        auto ret = group_gcode_commands(simple_gcode_multiple_groups_feedrate_shold_be_recorded_correctly,{{'F',3}});
        auto ret_array = std::vector<program_t>(ret.begin(),ret.end());
        INFO(back_to_gcode(ret));
        REQUIRE(ret.size() == 3);
        REQUIRE(ret_array.at(0).size() == 2);
        REQUIRE(ret_array.at(1).size() == 1);
        REQUIRE(ret_array.at(2).size() == 3);
        REQUIRE(ret_array.at(0).at(0).at('F') == Approx(10));
        REQUIRE(ret_array.at(2).at(0).at('F') == Approx(10));
    }
    

    SECTION("unsupported gcode must throw exception when there is no certainity about if G0 or G1 was used")
    {
        REQUIRE_THROWS(group_gcode_commands(simple_gcode_one_group_g1_w_error,{{'F',3}}));
        REQUIRE_THROWS(group_gcode_commands(simple_gcode_one_group_g1m_w_error,{{'F',3}}));
        
        REQUIRE_NOTHROW(group_gcode_commands(simple_gcode_one_group_g1_wo_error,{{'F',3}}));
    }

//     SECTION("simple G0 command with movement along X")
//     {
//         auto ret = to_vector(generate_path_intent({
//             { {'G',0.0}, {'X',10.0} }
//         }));
// 
//         REQUIRE(ret.size() == 3);
//         REQUIRE(std::get<distance_t>(ret.at(0)) == distance_t{0.0,0.0,0.0,0.0});
//         REQUIRE(std::get<movement::path_intentions::move_t>(ret.at(1)) == Approx(100));
//         REQUIRE(std::get<distance_t>(ret.at(2)) == distance_t{10.0,0.0,0.0,0.0});
//     }

}
