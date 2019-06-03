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

TEST_CASE("gcode_interpreter_test - merge_blocks", "[gcd][gcode_interpreter][merge_blocks]")
{

    SECTION("empty gives empty")
    {
        auto ret = merge_blocks({},{});
        REQUIRE(ret.size() == 0);
    }

    SECTION("one full and another empty gives one of them")
    {
        auto ret1 = merge_blocks({},{{'X',10}});
        REQUIRE(ret1.size() == 1);
        auto ret2 = merge_blocks({{'X',10}},{});
        REQUIRE(ret2.size() == 1);
        auto ret3 = merge_blocks({{'Y',3}},{{'X',10}});
        REQUIRE(ret3.size() == 2);
    }
    SECTION("merge two blocks")
    {
        auto ret1 = merge_blocks({{'X',3}},{{'X',10}});
        REQUIRE(ret1.size() == 1);
        REQUIRE(ret1['X'] == Approx(10));
        
        auto ret2 = merge_blocks({{'X',10}},{});
        REQUIRE(ret2.size() == 1);
        REQUIRE(ret2['X'] == Approx(10));

        auto ret3 = merge_blocks({{'Y',3},{'X',5}},{{'X',10}});
        REQUIRE(ret3.size() == 2);
        REQUIRE(ret3['X'] == Approx(10));
        REQUIRE(ret3['Y'] == Approx(3));
    }
}
