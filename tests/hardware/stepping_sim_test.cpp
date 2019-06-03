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
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/stepping.hpp>

#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <chrono>
#include <thread>
#include <vector>

using namespace raspigcd;
using namespace raspigcd::configuration;
using namespace raspigcd::hardware;


TEST_CASE("Hardware stepping_sim", "[hardware_stepping][stepping_sim]")
{
    stepping_sim worker({0, 0, 0, 0});

    SECTION("Run empty program")
    {
        int n = 0;
        worker.set_callback([&](const auto&) { n++; });
        worker.exec({});
        REQUIRE(n == 0);
    }

    SECTION("Run one command program")
    {
        int n = 0;
        worker.set_callback([&](const auto&) { n++; });
        single_step_command c = {.step = 0, .dir = 0};
        worker.exec({{{
            c, 
            c, 
            c, 
            c            
            }, 1}});
        REQUIRE(n == 1);
    }
    SECTION("Run one step in each positive  direction")
    {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 1;
            worker.set_callback([&](const auto&) { n++; });
            worker.current_steps = {5, 6, 7, 8};
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {5, 6, 7, 8};
            cmpto[i] += 1;
            REQUIRE(worker.current_steps == cmpto);
        }
    }
    SECTION("Run one step in each negative direction")
    {
        for (int i = 0; i < 4; i++) {
            int n = 0;
            multistep_command cmnd;
            cmnd.count = 1;
            cmnd.b[0].step = 0;
            cmnd.b[0].dir = 0;
            cmnd.b[1].step = 0;
            cmnd.b[1].dir = 0;
            cmnd.b[2].step = 0;
            cmnd.b[2].dir = 0;
            cmnd.b[3].step = 0;
            cmnd.b[3].dir = 0;
            cmnd.b[i].step = 1;
            cmnd.b[i].dir = 0;
            worker.set_callback([&](const auto&) { n++; });
            worker.current_steps = {1, 2, 3, 4};
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {1, 2, 3, 4};
            cmpto[i] -= 1;
            REQUIRE(worker.current_steps == cmpto);
        }
    }
}
