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

// 1 - output, 0 - input
// inspired by https://github.com/RPi-Distro/raspi-gpio/blob/master/raspi-gpio.c#L302 by Ja
void set_gpio_mode_x(struct bcm2835_peripheral& gpio_, int gpio, int fsel)
{
    uint32_t reg = gpio / 10;
    uint32_t sel = gpio % 10;
    volatile uint32_t* tmp = gpio_.addr + reg;
    *tmp = *tmp & (~(0x7 << (3 * sel))); // with mask
    *tmp = *tmp | ((fsel & 0x7) << (3 * sel));
}


raspberry_pi_3::raspberry_pi_3(const configuration::global& configuration)
{
    for (std::size_t i = 0; i < 5; i++)
        steps_counter[i] = 0;
    std::map<int, std::string> pins_taken;
    auto pins_taken_check = [&pins_taken](int p, std::string desc) {
        if (pins_taken.count(p)) throw std::invalid_argument(std::string("pin ") + std::to_string(p) + " already taken by " + pins_taken[p]);
        pins_taken[p] = desc;
    };
    // setup GPIO memory access
    gpio = {GPIO_BASE, 0, 0};
    std::cerr << "raspberry_pi_3::raspberry_pi_3: IO " << std::endl;
    // Open /dev/mem
    if ((gpio.mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw std::runtime_error(
            "Failed to open /dev/mem, try checking permissions.\n");
    }

    std::cerr << "raspberry_pi_3::raspberry_pi_3: MMAP " << std::endl;
    auto map_ = mmap(
        NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
        gpio.mem_fd, // File descriptor to physical memory virtual file '/dev/mem'
        gpio.addr_p  // Address in physical map that we want this memory block to
                     // expose
    );

    std::cerr << "raspberry_pi_3::raspberry_pi_3: GPIOADDR " << std::endl;
    if (map_ == MAP_FAILED) {
        throw std::runtime_error("map_peripheral failed");
    }

    gpio.addr = (volatile uint32_t*)map_;

    // setup configuration variables

    spindles = configuration.spindles;
    steppers = configuration.steppers;
    if (steppers.size() > 3) throw std::invalid_argument("raspberry_pi_3::raspberry_pi_3: currently the maximal number of stepper motors is 3.");
    buttons = configuration.buttons;

    std::cerr << "raspberry_pi_3::raspberry_pi_3: STEPPERS " << std::endl;
    // enable steppers
    for (auto c : steppers) {
        //        set_gpio_mode(gpio,c.step,1);
        //        set_gpio_mode(gpio,c.dir,1);
        //        set_gpio_mode(gpio,c.en,1);
        pins_taken_check(c.step, "OUT step");
        INP_GPIO(c.step);
        OUT_GPIO(c.step);
        pins_taken_check(c.dir, "OUT dir");
        INP_GPIO(c.dir);
        OUT_GPIO(c.dir);
        //pins_taken_check(c.en,"OUT en");
        INP_GPIO(c.en);
        OUT_GPIO(c.en);
    }
    std::cerr << "raspberry_pi_3::raspberry_pi_3: ENABLE_STEPPERS " << std::endl;
    enable_steppers({false, false, false, false});

    // enable spindles

    std::cerr << "raspberry_pi_3::raspberry_pi_3: SPINDLES " << spindles.size() << std::endl;
    _threads_alive = true;
    for (unsigned i = 0; i < spindles.size(); i++) {
        auto sppwm = spindles[i];
        std::cout << std::endl
                  << "setting pin " << sppwm.pin << " as spindle pwm output" << std::endl;
        //        set_gpio_mode(gpio,sppwm.pin,1);
        pins_taken_check(sppwm.pin, "OUT pwm spindle");
        INP_GPIO(sppwm.pin);
        OUT_GPIO(sppwm.pin);

        _spindle_duties.push_back(0.0);
        spindle_pwm_power(i, 0.0);
        _spindle_threads.push_back(std::thread([this, sppwm, i]() {
            std::cout << "starting spindle " << i << " thread" << std::endl;
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
                    if (sppwm.pin_negate)
                        GPIO_CLR = 1 << sppwm.pin;
                    else
                        GPIO_SET = 1 << sppwm.pin;
                    std::this_thread::sleep_until(prevTime + std::chrono::microseconds((int)(_duty * 1000000.0)));
                }
                if (_duty < sppwm.cycle_time_seconds) {
                    // 0
                    if (sppwm.pin_negate)
                        GPIO_SET = 1 << sppwm.pin;
                    else
                        GPIO_CLR = 1 << sppwm.pin;
                }
                prevTime = prevTime + std::chrono::microseconds((int)(sppwm.cycle_time_seconds * 1000000.0));
                std::this_thread::sleep_until(prevTime);
            }
            std::cout << "finished spindle " << i << " thread" << std::endl;
        }));
    }

    //    std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::seconds(3));

    // enable buttons

    using namespace std::chrono_literals;

    unsigned int pull_value = 0;
    for (auto& e : buttons) {
        pins_taken_check(e.pin, "IN button");
        INP_GPIO(e.pin);
        //        set_gpio_mode(gpio,e.pin,0);
        if (e.pullup) pull_value |= 1 << e.pin;
        buttons_state.push_back(0);
        buttons_callbacks.push_back([](int, int) {});
    }
    while (buttons_callbacks.size() < 100) {
        buttons_callbacks.push_back([](int, int) {});
    }

    // enable pull-up on selected gpios
    GPIO_PULL = 2;
    std::this_thread::sleep_for(10us);

    GPIO_PULLCLK0 = pull_value;
    std::this_thread::sleep_for(10us);

    GPIO_PULL = 0;
    GPIO_PULLCLK0 = 0;

    _btn_thread = std::async(std::launch::async, [this]() {
        static int anti_bounce_n = 100;
        std::vector<int> button_anti_bounce(buttons.size());
        while (_threads_alive) {
            using namespace std::chrono_literals;
            for (unsigned k_i = 0; k_i < buttons.size(); k_i++) {
                if (button_anti_bounce[k_i] > 0)
                    button_anti_bounce[k_i]--;
                else {
                    auto e = buttons[k_i];
                    int v = (unsigned char)(1 - ((GPIO_READ(e.pin)) >> (e.pin)));
                    v = (e.invert) ? (1 - v) : v;
                    if (buttons_state[k_i] != v) {
                        auto f = buttons_callbacks.at(k_i);
                        button_anti_bounce[k_i] = anti_bounce_n;
                        f(k_i, v);
                    }
                    buttons_state[k_i] = v;
                }
            }
            std::this_thread::sleep_for(200us);
        }
    });
}


void raspberry_pi_3::on_key(int btn, std::function<void(int, int)> callback_)
{
    buttons_callbacks[btn] = callback_;
}
std::function<void(int, int)> raspberry_pi_3::on_key(int btn)
{
    try {
        return buttons_callbacks.at(btn);
    } catch (...) {
        return [](int a, int b) { std::cout << "keybord handler not set " << a << " " << b << std::endl; };
    }
}
std::vector<int> raspberry_pi_3::keys_state()
{
    return buttons_state;
}


raspberry_pi_3::~raspberry_pi_3()
{
    _threads_alive = false;
    _btn_thread.get();
    for (auto& t : _spindle_threads)
        t.join(); //get();
    munmap((void*)gpio.addr, BLOCK_SIZE);
    close(gpio.mem_fd);
}


void raspberry_pi_3::do_step(const std::array<single_step_command, 4>& b)
{
    unsigned int step_clear = 0;
    // step direction
    unsigned int dir_set = 0;
    unsigned int dir_clear = 0;
    // shoud do step?
    unsigned int step_set = 0;
    for (std::size_t i = 0; i < steppers.size(); i++) {
        const auto& stepper = steppers[i];
        const auto& bs = b[i];
        step_clear = step_clear | (1 << stepper.step);
        // step direction
        dir_set = dir_set | (bs.dir << stepper.dir);
        dir_clear = dir_clear | ((1 - bs.dir) << stepper.dir);
        // shoud do step?
        step_set = step_set | (bs.step << stepper.step);
    }

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
    //{
    //    volatile int delayloop = 20;
    //    while (delayloop--)
    //        ;
    //}

    int lsteps_counter[5] = {steps_counter[0], steps_counter[1], steps_counter[2], steps_counter[3], 0};
    for (std::size_t i = 0; i < steppers.size(); i++) {
        const auto& bs = b[i];
        lsteps_counter[i] += ((((int)bs.dir) << 1) - 1)*(int)bs.step;
        lsteps_counter[4] = lsteps_counter[4] + lsteps_counter[i];
    }
    for (std::size_t i = 0; i < 5; i++)
        steps_counter[i] = lsteps_counter[i];
}

void raspberry_pi_3::enable_steppers(const std::vector<bool> en)
{
    _enabled_steppers = en;
    for (unsigned i = 0; i < std::min(en.size(), steppers.size()); i++) {
        auto c = steppers.at(i);
        if (en.at(i)) {
            GPIO_CLR = 1 << c.en;
        } else {
            GPIO_SET = 1 << c.en;
        }
    }
}

/**
     * @brief Get the steps counters for every stepper motor
     * 
     * @return std::vector<long int> number of steps for each motor
     */
steps_t raspberry_pi_3::get_steps() const
{
    while (true) {
    int lsteps_counter[5] = {steps_counter[0], steps_counter[1], steps_counter[2], steps_counter[3], steps_counter[4]};
    int chksum = 0;
    for (std::size_t i = 0; i < steppers.size(); i++) {
        chksum = chksum + lsteps_counter[i];
    }
    if (chksum == lsteps_counter[4]) {
        return {lsteps_counter[0],lsteps_counter[1],lsteps_counter[2],lsteps_counter[3]};
    }
    std::this_thread::yield();
    }
}

/**
     * @brief Set the steps counters
     * 
     * @param steps_count number of steps to reset to
     */
void raspberry_pi_3::set_steps(const steps_t steps_count__)
{
    int lsteps_counter[5] = {steps_count__[0], steps_count__[1], steps_count__[2], steps_count__[3], 0};
    for (std::size_t i = 0; i < steppers.size(); i++) {
        lsteps_counter[4] = lsteps_counter[4] + lsteps_counter[i];
    }
    for (std::size_t i = 0; i < 5; i++)
        steps_counter[i] = lsteps_counter[i];
}

void raspberry_pi_3::spindle_pwm_power(const int i, const double pwr0)
{
    auto pwr = pwr0;
    if (pwr < 0) throw std::invalid_argument("spindle power should be 0 or more");
    if (pwr > 1.1) throw std::invalid_argument("spindle power should be less or equal 1");
    if (pwr > 1.0) pwr = 1.0;
    _spindle_duties[i] = (spindles.at(i).duty_max - spindles.at(i).duty_min) * pwr + spindles.at(i).duty_min;
    std::cout << "sett duty to " << _spindle_duties[i] << std::endl;
}

} // namespace driver
} // namespace hardware
} // namespace raspigcd
