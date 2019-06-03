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


TEST_CASE("Hardware stepping_simple_timer", "[hardware_stepping][stepping_simple_timer]")
{
    std::shared_ptr<low_steppers> lsfake(new driver::inmem());
    std::shared_ptr<low_timers> ltfake = std::make_shared<driver::low_timers_fake>();
    stepping_simple_timer worker(60, lsfake, ltfake);

    SECTION("Run empty program")
    {
        int n = 0;
        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
        worker.exec({});
        REQUIRE(n == 0);
    }

    SECTION("Run one command program")
    {
        int n = 0;
        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
        single_step_command sc = {0,0};
        worker.exec({{.b = {sc, sc, sc, sc}, .count = 1}});
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
            ((driver::inmem*)lsfake.get())->current_steps = {5, 6, 7, 8};
            ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {5, 6, 7, 8};
            cmpto[i] += 1;
            REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
            REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == 1);
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
            ((driver::inmem*)lsfake.get())->current_steps = {1, 2, 3, 4};
            ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { n++; });
            worker.exec({cmnd});
            REQUIRE(n == 1);
            steps_t cmpto = {1, 2, 3, 4};
            cmpto[i] -= 1;
            REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
            REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == -1);
        }
    }

    SECTION("stepping break counter should be correct 1")
    {
        multistep_commands_t commands_to_do;

        for (int i = 0; i < 4; i++) {
            multistep_command cmnd;
            cmnd.count = 1 + i;
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
            commands_to_do.push_back(cmnd);
        }

        std::list<steps_t> steps_list = hardware_commands_to_steps(commands_to_do);
        steps_t last_step = hardware_commands_to_last_position_after_given_steps(commands_to_do, 1);
        REQUIRE(last_step == std::vector<steps_t>(steps_list.begin(), steps_list.end()).at(0));
    }

    SECTION("stepping break counter should be correct - 2")
    {
        multistep_commands_t commands_to_do;

        for (int i = 0; i < 4; i++) {
            multistep_command cmnd;
            cmnd.count = 1 + i;
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
            commands_to_do.push_back(cmnd);
        }

        std::list<steps_t> steps_list = hardware_commands_to_steps(commands_to_do);
        steps_t last_step = hardware_commands_to_last_position_after_given_steps(commands_to_do, 5);
        REQUIRE(last_step == std::vector<steps_t>(steps_list.begin(), steps_list.end()).at(4));
    }

    SECTION("stepping break counter should be correct - last")
    {
        multistep_commands_t commands_to_do;

        for (int i = 0; i < 4; i++) {
            multistep_command cmnd;
            cmnd.count = 1 + i;
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
            commands_to_do.push_back(cmnd);
        }

        std::list<steps_t> steps_list = hardware_commands_to_steps(commands_to_do);
        steps_t last_step = hardware_commands_to_last_position_after_given_steps(commands_to_do, -1);
        REQUIRE(last_step == steps_list.back());
    }

    SECTION("execute multiple commands in one program")
    {
        int n = 0;
        multistep_commands_t commands;
        for (int i = 0; i < 4; i++) {
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
            commands.push_back(cmnd);
        }

        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { 
            n++; 
        });
        worker.exec(commands);
        REQUIRE(n == 4);
        steps_t cmpto = {-1, -1, -1, -1};
        REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
        for (int i:{0,1,2,3})
        REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == -1);
    }

    SECTION("stop program after 2 steps then check position")
    {
        int n = 0;
        multistep_commands_t commands;
        for (int i = 0; i < 4; i++) {
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
            commands.push_back(cmnd);
        }

        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { 
            if (n == 1) {
                worker.terminate();
            }
            n++;
        });
        REQUIRE_THROWS_AS( worker.exec(commands), hardware::execution_terminated);
        REQUIRE(n == 2);
        steps_t cmpto = {-1, -1, 0, 0};
        REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
        for (int i:{0,1})
        REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == -1);
    }
    SECTION("stop program after 2 steps then check position - exception should have correct coordinates")
    {
        int n = 0;
        multistep_commands_t commands;
        for (int i = 0; i < 4; i++) {
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
            commands.push_back(cmnd);
        }

        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { 
            if (n == 1) {
                worker.terminate();
            }
            n++;
        });
        try {
            worker.exec(commands);
        }catch(const hardware::execution_terminated &e) {
            REQUIRE(e.delta_steps == ((driver::inmem*)lsfake.get())->current_steps);
        }
        REQUIRE(n == 2);
        steps_t cmpto = {-1, -1, 0, 0};
        REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
        for (int i:{0,1})
        REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == -1);
    }

    SECTION("Stop program in multiple steps")
    {
        int n = 0;
        multistep_commands_t commands;
        for (int i = 0; i < 40; i++) {
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
            cmnd.b[i%4].step = 1;
            cmnd.b[i%4].dir = 0;
            commands.push_back(cmnd);
        }

        ((driver::inmem*)lsfake.get())->current_steps = {0, 0, 0, 0};
        ((driver::inmem*)lsfake.get())->set_step_callback([&](const auto&) { 
            if (n == 1) {
                worker.terminate(16);
            }
            n++;
        });
        try {
            worker.exec(commands);
        }catch(const hardware::execution_terminated &e) {
            REQUIRE(e.delta_steps == ((driver::inmem*)lsfake.get())->current_steps);
        }
        REQUIRE(n == 18);
        steps_t cmpto = {-5, -5, -4, -4};
        REQUIRE(((driver::inmem*)lsfake.get())->current_steps == cmpto);
        for (int i:{0,1})
        REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == -5);
        for (int i:{2,3})
        REQUIRE(((driver::inmem*)lsfake.get())->counters[i] == -4);
    }

}
