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


// #define CATCH_CONFIG_DISABLE_MATCHERS
// #define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>
#include <chrono>
#include <condition_variable>
#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/driver/low_timers_wait_for.hpp>
#include <hardware/stepping.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/*
Terminate the execution of steps program

This tests both stepping_sim and stepping_simple_timer
*/


using namespace raspigcd;
using namespace raspigcd::hardware;
//using namespace raspigcd::movement;
using namespace raspigcd::configuration;


TEST_CASE("terminate procedure for stepping_simple_timer.", "[gcd][stepping_simple_timer][execute_pure_path_intent]")
{
        multistep_commands_t commands_to_do;
        for (int i = 0; i < 4; i++) {
            //int n = 0;
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
            commands_to_do.push_back(cmnd);
        }

    auto steppers_drv = std::make_shared<hardware::driver::inmem>();
    auto timers_drv = std::make_shared<hardware::driver::low_timers_fake>();

    std::shared_ptr<hardware::stepping> stepping = std::make_shared<hardware::stepping_simple_timer>(50, steppers_drv, timers_drv);


    hardware::driver::inmem* inmem_ptr = (hardware::driver::inmem*)steppers_drv.get();
    //hardware::driver::low_timers_fake* low_timers_fake_ptr = (hardware::driver::low_timers_fake*)timers_drv.get();
    std::mutex m;
    std::mutex n;

    SECTION("break execution and check that the steps performed are less than requested")
    {


        int i = 0;
        inmem_ptr->set_step_callback([&](const steps_t&) {
            if (i == 1) {
                m.unlock();
                n.lock();
            } // sync point after first step
            i++;
        });
        m.lock();
        n.lock();
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do);
            } catch (const raspigcd::hardware::execution_terminated& e) {
            }
        });
        m.lock();
        stepping.get()->terminate();
        n.unlock();

        worker.join();
        REQUIRE(stepping.get()->get_tick_index() == 2);
        REQUIRE(stepping.get()->get_tick_index() == i);

        std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
        std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(), steps_from_commands_.end());
        REQUIRE(inmem_ptr->current_steps == steps_from_commands[stepping.get()->get_tick_index() - 1]);
    }
    /*
    
    SECTION("execute not that simple program and check result")
    {
        int steps_counter = 0;
        std::mutex set_step_callback_mutex;
        std::mutex set_step_callback_mutex_2;
        inmem_ptr->set_step_callback([&](const steps_t&) {
            if (steps_counter == 100) {
                set_step_callback_mutex_2.unlock();
                set_step_callback_mutex.lock(); // here it will wait until lock is freed
            }
        });

        set_step_callback_mutex.lock();
        set_step_callback_mutex_2.lock();
        executor.set_gcode_interpreter_objects(objs);
        // it should stop after 100 steps
        auto handle = std::async(std::launch::async,[&](){
            executor.execute({
                distance_t{0,0,0,0},
                movement::path_intentions::move_t(48.0),
                distance_t{1,2,3,4},
                movement::path_intentions::move_t(20.0),
                distance_t{2,1,0,2}
            });
        });
        set_step_callback_mutex_2.lock(); // wait for thread to unlock
        // do the break!
        executor.break();
        set_step_callback_mutex.unlock(); // now we unlock
//        executor.get
//        REQUIRE(inmem_ptr->current_steps == objs.motor_layout.get()->cartesian_to_steps(distance_t{2,1,0,2}));
    }
    */
}


TEST_CASE("terminate procedure on stepping_sim for verification.", "[gcd][stepping_sim][execute_pure_path_intent][stepping_simple_timer]")
{
    multistep_commands_t commands_to_do;
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
        commands_to_do.push_back(cmnd);
    }
    std::mutex m;
    std::mutex n;
    SECTION("break execution and check that the steps performed are less than requested")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_sim>(
            steps_t{0, 0, 0, 0},
            [&](const steps_t& s) {
                if (i == 1) {
                    m.unlock();
                    n.lock();
                }
                current_steps = s;
                i++;
            });

        m.lock();
        n.lock();
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do);
            } catch (const raspigcd::hardware::execution_terminated& e) {
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate();
        n.unlock();

        worker.join();
        REQUIRE(stepping.get()->get_tick_index() == 2);
        REQUIRE(stepping.get()->get_tick_index() == i);
        std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
        std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(), steps_from_commands_.end());
        REQUIRE(current_steps == steps_from_commands[stepping.get()->get_tick_index() - 1]);
    }

    SECTION("break execution and check that the steps performed are less than requested for stepping simple")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<raspigcd::hardware::driver::inmem> low_steppers_drv_a = std::make_shared<raspigcd::hardware::driver::inmem>();
        std::shared_ptr<hardware::low_steppers> low_steppers_drv = low_steppers_drv_a;
        low_steppers_drv_a->set_step_callback([&](const steps_t& s) {
            if (i == 1) {
                m.unlock();
                n.lock();
            }
            current_steps = s;
            i++;
        });

        std::shared_ptr<hardware::low_timers> low_timers_drv = std::make_shared<raspigcd::hardware::driver::low_timers_wait_for>();
        // TODO
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_simple_timer>(
            100,
            low_steppers_drv,
            low_timers_drv);

        m.lock();
        n.lock();
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do);
            } catch (const raspigcd::hardware::execution_terminated& e) {
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate();
        n.unlock();

        worker.join();
        REQUIRE(stepping.get()->get_tick_index() == 2);
        REQUIRE(stepping.get()->get_tick_index() == i);
        std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
        std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(), steps_from_commands_.end());
        REQUIRE(current_steps == steps_from_commands[stepping.get()->get_tick_index() - 1]);
    }

    SECTION("callback function on break should be available")
    {
        steps_t current_steps;
        std::shared_ptr<raspigcd::hardware::driver::inmem> low_steppers_drv_a = std::make_shared<raspigcd::hardware::driver::inmem>();
        std::shared_ptr<hardware::low_steppers> low_steppers_drv = low_steppers_drv_a;
        low_steppers_drv_a->set_step_callback([&](const steps_t&) {});
        std::shared_ptr<hardware::low_timers> low_timers_drv = std::make_shared<raspigcd::hardware::driver::low_timers_wait_for>();
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_simple_timer>(100, low_steppers_drv, low_timers_drv);
        REQUIRE_NOTHROW(stepping.get()->exec(commands_to_do, [](const steps_t, const int ) {
            return 0;
        }));
    }


    SECTION("break execution - callback rules that exception should be throwed - stepping_sim")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_sim>(
            steps_t{0, 0, 0, 0},
            [&](const steps_t& s) {
                if (i == 1) {
                    m.unlock();
                    n.lock();
                }
                current_steps = s;
                i++;
            });

        m.lock();
        n.lock();
        bool throwed = false;
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do, [](const steps_t , const int ) {
                    return 0;
                });
            } catch (const raspigcd::hardware::execution_terminated& e) {
                throwed = true;
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate();
        n.unlock();

        worker.join();
        REQUIRE(throwed);
    }

    SECTION("break execution - callback rules that exception should be throwed - stepping_simple_timer")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<raspigcd::hardware::driver::inmem> low_steppers_drv_a = std::make_shared<raspigcd::hardware::driver::inmem>();
        std::shared_ptr<hardware::low_steppers> low_steppers_drv = low_steppers_drv_a;
        low_steppers_drv_a->set_step_callback([&](const steps_t& s) {
            if (i == 1) {
                m.unlock();
                n.lock();
            }
            current_steps = s;
            i++;
        });

        std::shared_ptr<hardware::low_timers> low_timers_drv = std::make_shared<raspigcd::hardware::driver::low_timers_wait_for>();
        // TODO
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_simple_timer>(
            100,
            low_steppers_drv,
            low_timers_drv);

        m.lock();
        n.lock();
        bool throwed = false;
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do, [](const steps_t , const int ) {
                    return 0;
                });
            } catch (const raspigcd::hardware::execution_terminated& e) {
                throwed = true;
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate();
        n.unlock();
        worker.join();
        REQUIRE(throwed);
    }


    SECTION("break execution - callback rules that exception should be throwed - stepping_sim")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_sim>(
            steps_t{0, 0, 0, 0},
            [&](const steps_t& s) {
                if (i == 1) {
                    m.unlock();
                    n.lock();
                }
                current_steps = s;
                i++;
            });

        m.lock();
        n.lock();
        bool throwed = false;
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do, [](const steps_t , const int ) {
                    return 1;
                });
            } catch (const raspigcd::hardware::execution_terminated& e) {
                throwed = true;
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate();
        n.unlock();

        worker.join();
        REQUIRE(!throwed);
    }

    SECTION("break execution - callback rules that exception should be throwed - stepping_simple_timer")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<raspigcd::hardware::driver::inmem> low_steppers_drv_a = std::make_shared<raspigcd::hardware::driver::inmem>();
        std::shared_ptr<hardware::low_steppers> low_steppers_drv = low_steppers_drv_a;
        low_steppers_drv_a->set_step_callback([&](const steps_t& s) {
            if (i == 1) {
                m.unlock();
                n.lock();
            }
            current_steps = s;
            i++;
        });

        std::shared_ptr<hardware::low_timers> low_timers_drv = std::make_shared<raspigcd::hardware::driver::low_timers_wait_for>();
        // TODO
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_simple_timer>(
            100,
            low_steppers_drv,
            low_timers_drv);

        m.lock();
        n.lock();
        bool throwed = false;
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do, [](const steps_t , const int ) {
                    return 1;
                });
            } catch (const raspigcd::hardware::execution_terminated& e) {
                throwed = true;
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate();
        n.unlock();
        worker.join();
        REQUIRE(!throwed);
    }



    SECTION("break execution with slowdown should do the exact number of steps before stop - stepping_simple_timer")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<raspigcd::hardware::driver::inmem> low_steppers_drv_a = std::make_shared<raspigcd::hardware::driver::inmem>();
        std::shared_ptr<hardware::low_steppers> low_steppers_drv = low_steppers_drv_a;
        low_steppers_drv_a->set_step_callback([&](const steps_t& s) {
            if (i == 1) {
                m.unlock();
                n.lock();
            }
            current_steps = s;
            i++;
        });

        std::list < double > low_timers_drv_history;
        std::shared_ptr<hardware::low_timers> low_timers_drv = 
        std::make_shared<raspigcd::hardware::driver::low_timers_fake>([&](auto t){
            low_timers_drv_history.push_back(t);
        });
        // TODO
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_simple_timer>(
            1000,
            low_steppers_drv,
            low_timers_drv);

        m.lock();
        n.lock();
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do);
            } catch (const raspigcd::hardware::execution_terminated& e) {
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate(5);
        n.unlock();

        worker.join();
        REQUIRE(stepping.get()->get_tick_index() == 2+5);
        REQUIRE(stepping.get()->get_tick_index() == i);
        std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
        std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(), steps_from_commands_.end());
        REQUIRE(current_steps == steps_from_commands[stepping.get()->get_tick_index() - 1]);
        
        auto expected_low_timers_drv_history = low_timers_drv_history;
        expected_low_timers_drv_history = {1000,1000,1001,1002,1003,1004,1005};
        REQUIRE(low_timers_drv_history == expected_low_timers_drv_history);
    }

    SECTION("break and continue execution with slowdown should do the exact timings - stepping_simple_timer")
    {
        int i = 0;

        steps_t current_steps;
        std::shared_ptr<raspigcd::hardware::driver::inmem> low_steppers_drv_a = std::make_shared<raspigcd::hardware::driver::inmem>();
        std::shared_ptr<hardware::low_steppers> low_steppers_drv = low_steppers_drv_a;
        low_steppers_drv_a->set_step_callback([&](const steps_t& s) {
            if (i == 1) {
                m.unlock();
                n.lock();
            }
            current_steps = s;
            i++;
        });

        std::list < double > low_timers_drv_history;
        std::shared_ptr<hardware::low_timers> low_timers_drv = 
        std::make_shared<raspigcd::hardware::driver::low_timers_fake>([&](auto t){
            low_timers_drv_history.push_back(t);
        });
        // TODO
        std::shared_ptr<hardware::stepping> stepping = std::make_shared<raspigcd::hardware::stepping_simple_timer>(
            1000,
            low_steppers_drv,
            low_timers_drv);

        m.lock();
        n.lock();
        std::thread worker([&]() {
            try {
                stepping.get()->exec(commands_to_do,[&](auto , auto ){
                    return 1;
                });
            } catch (const raspigcd::hardware::execution_terminated& e) {
            }
        });
        //worker.detach();
        m.lock();
        stepping.get()->terminate(5);
        n.unlock();

        worker.join();
        REQUIRE(stepping.get()->get_tick_index() == 40);
        REQUIRE(stepping.get()->get_tick_index() == i);
        std::list<steps_t> steps_from_commands_ = hardware_commands_to_steps(commands_to_do);
        std::vector<steps_t> steps_from_commands(steps_from_commands_.begin(), steps_from_commands_.end());
        REQUIRE(current_steps == steps_from_commands[stepping.get()->get_tick_index() - 1]);
        
        auto expected_low_timers_drv_history = low_timers_drv_history;
        expected_low_timers_drv_history = {1000,1000,1001,1002,1003,1004,1005,1005,1004,1003,1002,1001};
        while (expected_low_timers_drv_history.size() < 40) {
            expected_low_timers_drv_history.push_back(1000);
        }
        REQUIRE(low_timers_drv_history == expected_low_timers_drv_history);
    }


}
