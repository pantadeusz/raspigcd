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

#define CATCH_CONFIG_DISABLE_MATCHERS
#define CATCH_CONFIG_FAST_COMPILE
#include <catch2/catch.hpp>

#include <vector>
#include <chrono>
#include <thread>

using namespace raspigcd;
using namespace raspigcd::configuration;


TEST_CASE( "Configuration handling", "[configuration]" ) {

    SECTION( "configuration object is created correctly" ) {
        raspigcd::configuration::global cfg;
        //REQUIRE(&cfg != nullptr);
    }

    SECTION( "configuration method load_defaults returns the same object" ) {
        raspigcd::configuration::global cfg;
        REQUIRE(&(cfg.load_defaults()) == &cfg);
    }

    SECTION( "operator equals should work" ) {
        raspigcd::configuration::global cfg_orig;
        
    cfg_orig.tick_duration_us = 100;
    cfg_orig.simulate_execution = false;
    cfg_orig.scale = {1.0,1.0,1.0,1.0}; 
    cfg_orig.max_accelerations_mm_s2 =  {1000,1000,1000,1000};
    cfg_orig.max_velocity_mm_s = {100,100,100,100};
    cfg_orig.max_no_accel_velocity_mm_s = {5.0,5.0,5.0,5.0};
    cfg_orig.motion_layout = configuration::motion_layouts::COREXY;
    cfg_orig.steppers = {stepper(27, 10, 22, 100.0),stepper(4, 10, 17, 100.0),stepper(9, 10, 11, 100.0),stepper(0, 10, 5, 100.0)};
    cfg_orig.buttons = {{.pin = 21, .pullup = true}, {.pin = 20, .pullup = true}, {.pin = 16, .pullup = true}, {.pin = 12, .pullup = true}};
    cfg_orig.spindles = {
        {
            .pin = 18,
            .cycle_time_seconds = 0.1,
            .duty_min = 0.0,
            .duty_max = 0.1
        }
    };

        global cfg_new = cfg_orig; REQUIRE(cfg_new == cfg_orig);
        cfg_new = cfg_orig; cfg_new.tick_duration_us = 22; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.simulate_execution = true; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.scale[2] = -5; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.max_accelerations_mm_s2[0] = 12; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.max_velocity_mm_s[1] = 90; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.max_no_accel_velocity_mm_s[0]=1000; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.motion_layout = configuration::motion_layouts::CARTESIAN; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.spindles[0].pin = 1; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.steppers[1].en = 9; REQUIRE(!(cfg_new == cfg_orig));
        cfg_new = cfg_orig; cfg_new.buttons[0].pin = 9; REQUIRE(!(cfg_new == cfg_orig));

    }

    SECTION( "distance_t check" ) {
        distance_t a = {1,2,3,4};
        distance_t b;
        b = a;
        REQUIRE(a == b);
    }


    SECTION( "json conversions" ) {
        raspigcd::configuration::global cfg1;
        cfg1.tick_duration_us = 0;
        cfg1.simulate_execution = true;
        cfg1.load_defaults();
        nlohmann::json j1 = cfg1;
        raspigcd::configuration::global cfg2;
        cfg2 = j1;
        REQUIRE(cfg1 == cfg2);
    }


    //     conf &load(const std::string &filename);
//     conf &save(const std::string &filename);
    SECTION( "configuration equal operator" ) {
        raspigcd::configuration::global cfg;
        cfg.load_defaults();
        raspigcd::configuration::global cfg2;
        INFO(cfg);
        INFO(cfg2);
        REQUIRE(!(cfg2 == cfg));
        cfg2.load_defaults();
        REQUIRE(cfg2 == cfg);
    }

    SECTION( "configuration method save and load works as expected returns the same object" ) {
        raspigcd::configuration::global cfg;
        cfg.load_defaults().save("__test_tmp.cfg");
        raspigcd::configuration::global cfg2;
        cfg2.load("__test_tmp.cfg");
        REQUIRE(cfg2 == cfg);
    }

    SECTION( "tick duration should be correctly calculated" ) {
        raspigcd::configuration::global cfg;
        cfg.tick_duration_us = 60;
        REQUIRE(cfg.tick_duration() == Approx( 0.00006 ));
        cfg.tick_duration_us = 100;
        REQUIRE(cfg.tick_duration() == Approx( 0.0001 ));
    }

    SECTION( "wrong layout should throw exception " ) {
        nlohmann::json j = {{"motion_layout","some wrong layout"}};
        raspigcd::configuration::global cfg;
        REQUIRE_THROWS([&](){cfg = j;}(), 
            std::invalid_argument("motion_layout can be only corexy or cartesian")
        );
    }
    
    SECTION( "corexy support " ) {
        nlohmann::json j = {{"motion_layout","corexy"}};
        raspigcd::configuration::global cfg;
        cfg = j;
        REQUIRE(cfg.motion_layout == raspigcd::configuration::motion_layouts::COREXY);
    }

    SECTION( "cartesian support" ) {
        nlohmann::json j = {{"motion_layout","cartesian"}};
        raspigcd::configuration::global cfg;
        cfg = j;
        REQUIRE(cfg.motion_layout == raspigcd::configuration::motion_layouts::CARTESIAN);
    }
  
}
