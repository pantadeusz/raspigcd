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




#include <hardware/driver/raspberry_pi.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
// RASPBERRY PI GPIO - docs from http://www.pieter-jan.com/node/15

#include <stdio.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>

// #define BCM2708_PERI_BASE       0x20000000   // raspi 1 //
#define BCM2708_PERI_BASE 0x3F000000 // raspi 3 //
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)

#define BLOCK_SIZE (4 * 1024)

#define INP_GPIO(g) *(gpio.addr + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define OUT_GPIO(g) *(gpio.addr + ((g) / 10)) |= (1 << (((g) % 10) * 3))
#define SET_GPIO_ALT(g, a)         \
    *(gpio.addr + (((g) / 10))) |= \
        (((a) <= 3 ? (a) + 4 : (a) == 4 ? 3 : 2) << (((g) % 10) * 3))

#define GPIO_SET \
    *(gpio.addr + 7) // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR \
    *(gpio.addr + 10) // clears bits which are 1 ignores bits which are 0

#define GPIO_READ(g) *(gpio.addr + 13) &= (1 << (g))

#define GET_GPIO(g) (*(gpio.addr + 13) & (1 << g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio.addr + 37)     // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio.addr + 38) // Pull up/pull down clock

namespace raspigcd {
namespace hardware {

namespace driver {

raspberry_pi_3::raspberry_pi_3(const configuration::global& configuration)
{
    // setup GPIO memory access
    gpio = {GPIO_BASE, 0, 0, 0};
    // Open /dev/mem
    if ((gpio.mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw std::runtime_error(
            "Failed to open /dev/mem, try checking permissions.\n");
    }

    gpio.map = mmap(
        NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
        gpio.mem_fd, // File descriptor to physical memory virtual file '/dev/mem'
        gpio.addr_p  // Address in physical map that we want this memory block to
                     // expose
    );

    if (gpio.map == MAP_FAILED) {
        throw std::runtime_error("map_peripheral failed");
    }

    gpio.addr = (volatile unsigned int*)gpio.map;

    // setup configuration variables

    spindles = configuration.spindles;
    steppers = configuration.steppers;
    buttons = configuration.buttons;

    // enable steppers
    for (auto c : steppers) {
        INP_GPIO(c.step);
        OUT_GPIO(c.step);
        INP_GPIO(c.dir);
        OUT_GPIO(c.dir);
        INP_GPIO(c.en);
        OUT_GPIO(c.en);
    }
    enable_steppers({false, false, false, false});

    // enable spindles

    _threads_alive = true;
    for (unsigned i = 0; i < spindles.size(); i++) {
        auto sppwm = spindles[i];
        std::cout << "setting pin " << sppwm.pin << " as spindle pwm output" << std::endl;
        INP_GPIO(sppwm.pin);
        OUT_GPIO(sppwm.pin);
        _spindle_duties.push_back(0.0);
        spindle_pwm_power(i, 0.0);
        _spindle_threads.push_back(std::thread([this, sppwm, i]() {
            double& _duty = _spindle_duties[i];
            /* {
                sched_param sch_params;
                sch_params.sched_priority = sched_get_priority_max(SCHED_RR);

                if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch_params)) {
                    std::cerr << "Warning: spindle_pi::configure - set realtime thread failed" << std::endl;
                }
            } */
            auto prevTime = std::chrono::steady_clock::now();
            while (_threads_alive) {
                // 1
                if (_duty >= 0.0) {
                    GPIO_SET = 1 << sppwm.pin;
                    std::this_thread::sleep_until(prevTime + std::chrono::microseconds((int)(_duty * 1000000.0)));
                }
                if (_duty < sppwm.cycle_time_seconds) {
                    // 0
                    GPIO_CLR = 1 << sppwm.pin;
                }
                prevTime = prevTime + std::chrono::microseconds((int)(sppwm.cycle_time_seconds * 1000000.0));
                std::this_thread::sleep_until(prevTime);
            }
        }));
    }

    //    std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::seconds(3));

    // enable buttons

    using namespace std::chrono_literals;

    unsigned int pull_value = 0;
    for (auto& e : buttons) {
        INP_GPIO(e.pin);
        if (e.pullup) pull_value |= 1 << e.pin;
        buttons_state.push_back(0);
        buttons_callbacks.push_back([](int,int){});
    }

    // enable pull-up on selected gpios
    GPIO_PULL = 2;
    std::this_thread::sleep_for(10us);

    GPIO_PULLCLK0 = pull_value;
    std::this_thread::sleep_for(10us);

    GPIO_PULL = 0;
    GPIO_PULLCLK0 = 0;

    _btn_thread = std::thread([this]() {
        while (_threads_alive) {
            using namespace std::chrono_literals;
            for (unsigned k_i = 0; k_i < buttons.size(); k_i++) {
                auto e = buttons[k_i];
                int v = (unsigned char)(1 - ((GPIO_READ(e.pin)) >> (e.pin)));
                //if (buttons_state.count(k_i)) {
                    if (buttons_state[k_i] != v) {
                        // // button down
                        // if (v > 0) kv.second(k_i);
                        // // button up
                        // // if (v <= 0) kv.second (kv.first);
                        auto f = buttons_callbacks.at(k_i);
                        f(k_i,v);
                    }
                //}
                buttons_state[k_i] = v;
            }
            std::this_thread::sleep_for(10ms);
        }
    });
}


void raspberry_pi_3::on_key(int btn, std::function<void(int,int)> callback_) {
    buttons_callbacks.at(btn) = callback_;
}
std::function<void(int,int)>  raspberry_pi_3::on_key(int btn) {
    return buttons_callbacks.at(btn);
}
std::vector < int > raspberry_pi_3::keys_state() {
    return buttons_state;
}


raspberry_pi_3::~raspberry_pi_3()
{
    _threads_alive = false;
    _btn_thread.join();
    for (auto& t : _spindle_threads)
        t.join();
    munmap(gpio.map, BLOCK_SIZE);
    close(gpio.mem_fd);
}


void raspberry_pi_3::do_step(const std::array<single_step_command,4> &b)
{
    unsigned int step_clear = (1 << steppers[0].step) | (1 << steppers[1].step) |
                              (1 << steppers[2].step) | (1 << steppers[3].step);
    // step direction
    unsigned int dir_set =
        (b[0].dir << steppers[0].dir) | (b[1].dir << steppers[1].dir) |
        (b[2].dir << steppers[2].dir) | (b[3].dir << steppers[3].dir);
    unsigned int dir_clear = ((1 - b[0].dir) << steppers[0].dir) |
                             ((1 - b[1].dir) << steppers[1].dir) |
                             ((1 - b[2].dir) << steppers[2].dir) |
                             ((1 - b[3].dir) << steppers[3].dir);
    // shoud do step?
    unsigned int step_set =
        (b[0].step << steppers[0].step) | (b[1].step << steppers[1].step) |
        (b[2].step << steppers[2].step) | (b[3].step << steppers[3].step);

    // first set directions
    GPIO_SET = dir_set;
    GPIO_CLR = dir_clear;
    {
        volatile int delayloop = 50;
        while (delayloop--)
            ;
    }
    // set step to do
    GPIO_SET = step_set;
    {
        volatile int delayloop = 100;
        while (delayloop--)
            ;
    }
    // clear all step pins
    GPIO_CLR = step_clear;
    {
        volatile int delayloop = 20;
        while (delayloop--)
            ;
    }
}

void raspberry_pi_3::enable_steppers(const std::vector<bool> en)
{
    _enabled_steppers = en;
    for (unsigned i = 0; i < en.size(); i++) {
        auto c = steppers.at(i);
        if (en.at(i)) {
            GPIO_CLR = 1 << c.en;
        } else {
            GPIO_SET = 1 << c.en;
        }
    }
}


void raspberry_pi_3::spindle_pwm_power(const int i, const double pwr0)
{
    auto pwr = pwr0;
    if (pwr < 0) throw std::invalid_argument("spindle power should be 0 or more");
    if (pwr > 1.1) throw std::invalid_argument("spindle power should be less or equal 1");
    if (pwr > 1.0) pwr = 1.0;
    _spindle_duties[i] = (spindles.at(i).duty_max - spindles.at(i).duty_min) * pwr + spindles.at(i).duty_min;
}

} // namespace driver
} // namespace hardware
} // namespace raspigcd
