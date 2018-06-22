/*

    Raspberry Pi G-CODE interpreter
    Copyright (C) 2018  Tadeusz Puźniakowski puzniakowski.pl

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

#include "m_motor_rpi.hpp"

#include <chrono>

#include <chrono>
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

// IO Acces
struct bcm2835_peripheral {
    unsigned long addr_p;
    int mem_fd;
    void* map;
    volatile unsigned int* addr;
};

struct bcm2835_peripheral gpio = { GPIO_BASE, 0, 0, 0 };

// Exposes the physical address defined in the passed structure using mmap on /dev/mem
int map_peripheral(struct bcm2835_peripheral* p)
{
    // Open /dev/mem
    if ((p->mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        throw std::runtime_error("Failed to open /dev/mem, try checking permissions.\n");
    }

    p->map = mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        p->mem_fd, // File descriptor to physical memory virtual file '/dev/mem'
        p->addr_p // Address in physical map that we want this memory block to expose
        );

    if (p->map == MAP_FAILED) {
        throw std::runtime_error("map_peripheral failed");
    }

    p->addr = (volatile unsigned int*)p->map;

    return 0;
}

void unmap_peripheral(struct bcm2835_peripheral* p)
{
    munmap(p->map, BLOCK_SIZE);
    close(p->mem_fd);
}

#define INP_GPIO(g) *(gpio.addr + ((g) / 10)) &= ~(7 << (((g) % 10) * 3))
#define OUT_GPIO(g) *(gpio.addr + ((g) / 10)) |= (1 << (((g) % 10) * 3))
#define SET_GPIO_ALT(g, a) *(gpio.addr + (((g) / 10))) |= (((a) <= 3 ? (a) + 4 : (a) == 4 ? 3 : 2) << (((g) % 10) * 3))

#define GPIO_SET *(gpio.addr + 7) // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio.addr + 10) // clears bits which are 1 ignores bits which are 0

#define GPIO_READ(g) *(gpio.addr + 13) &= (1 << (g))

#define GET_GPIO(g) (*(gpio.addr + 13) & (1 << g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio.addr + 37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio.addr + 38) // Pull up/pull down clock

namespace tp {
namespace motor {

    class stepper_pi : public i_Stepper {
    protected:
        std::vector<StepperPiConfig> conf;
        bool enabled_;

    public:
        stepper_pi();
        /**
	    * Setup stepper motor - selects pins for motor and return 0 if success
	    */
        int configure(const std::vector<StepperPiConfig>& conf);
        /**
	    * executes one motor step. takes value - number of steps to perform in given direction (positive or negative). Warning - for smooth work it should be max 1 step
	    */
        void step(std::array<signed char, 4>& dir);
        /**
	    * enable or disable stepper motor
	    */
        void enabled(bool en);

        bool is_enabled();

        virtual ~stepper_pi()
        {
        }
    };

    class spindle_pi : public i_Spindle {
    protected:
        SpindlePiConfig conf;
        int dutyCycle; // 0..2000 gdzie 0 - brak sygnalu, 1000 min 2000 max
        bool spindleAlive;

    public:
        spindle_pi();
        /**
	    * Setup stepper motor - selects pins for motor and return 0 if success
	    */
        int configure(const SpindlePiConfig& conf);

        void setSpeed(double v);
        virtual ~spindle_pi()
        {
        }
    };

    class buttons_pi : public i_Buttons {
    protected:
        ButtonsPiConfig config;

    public:
        buttons_pi();
        int configure(const ButtonsPiConfig& c);
        std::array<unsigned char, 4> getButtons();
    };

    int piConfigured = 0;
    int configure_pi()
    {
        if (piConfigured == 1)
            return 0;
        if (map_peripheral(&gpio) == -1) {
            printf("Failed to map the physical GPIO registers into the virtual memory space.\n");
            return -1;
        }
        piConfigured = 1;
        return 0;
    }

    stepper_pi::stepper_pi()
    {
        static int configured = configure_pi();
        if (configured < 0)
            throw "could not setup raspberry pi gpio";
    }

    int stepper_pi::configure(const std::vector<StepperPiConfig>& conf)
    {
        this->conf = conf;
        // Define pin 7 as output
        for (auto c : conf) {
            INP_GPIO(c.step);
            OUT_GPIO(c.step);
            INP_GPIO(c.dir);
            OUT_GPIO(c.dir);
            INP_GPIO(c.en);
            OUT_GPIO(c.en);
        }
        enabled(false);
        return 0;
    }

    void stepper_pi::step(std::array<signed char, 4>& dirs)
    {
        if (dirs[3] != 0)
            throw "this should not happen";
        for (unsigned i = 0; i < dirs.size(); i++) {
            if (dirs[i] > 0) {
                GPIO_SET = 1 << conf[i].dir;
                dirs[i]--;
                GPIO_SET = 1 << conf[i].step;
            } else if (dirs[i] < 0) {
                GPIO_CLR = 1 << conf[i].dir;
                dirs[i]++;
                GPIO_SET = 1 << conf[i].step;
            };
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        for (unsigned i = 0; i < dirs.size(); i++) {
            GPIO_CLR = 1 << conf[i].step;
        }
    }
    void stepper_pi::enabled(bool en)
    {
        for (auto c : conf) {
            //std::cout << "en["<<conf.en<<"]: " << en << std::endl;
            if (en) {
                GPIO_CLR = 1 << c.en;
            } else {
                GPIO_SET = 1 << c.en;
            }
        }
        this->enabled_ = en;
    }
    bool stepper_pi::is_enabled()
    {
        return enabled_;
    }

    spindle_pi::spindle_pi()
    {
        static int configured = configure_pi();
        if (configured < 0)
            throw "could not setup raspberry pi gpio";
    }

    int spindle_pi::configure(const SpindlePiConfig& c)
    {
        this->conf = c;
        INP_GPIO(c.pin);
        OUT_GPIO(c.pin);

        std::cout << "servopwm: " << conf.servopwm << std::endl;
        if (conf.servopwm == 1) {
            dutyCycle = 0;
            spindleAlive = true;
            std::thread t([this]() {
                {
                    sched_param sch_params;
                    sch_params.sched_priority = sched_get_priority_max(SCHED_RR);

                    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch_params)) {
                        std::cerr << "Warning: spindle_pi::configure - set realtime thread failed" << std::endl;
                    }
                }
                auto prevTime = std::chrono::steady_clock::now();
                while (spindleAlive) {
                    // 1
                    GPIO_SET = 1 << conf.pin;
                    std::this_thread::sleep_until(prevTime + std::chrono::microseconds(dutyCycle));
                    GPIO_CLR = 1 << conf.pin;
                    // 0
                    prevTime = prevTime + std::chrono::microseconds(20000);
                    std::this_thread::sleep_until(prevTime);
                }
            });
            t.detach();
        } else {
            spindleAlive = true;
        }
        setSpeed(0.0);

        std::this_thread::sleep_until(std::chrono::steady_clock::now() + std::chrono::seconds(3));

        return 0;
    }

    void spindle_pi::setSpeed(double v)
    {
        if (conf.servopwm == 1) {
            if (v < -1)
                dutyCycle = 0;
            else if (v < 0)
                dutyCycle = 1000;
            else if (v <= 1.0)
                dutyCycle = (int)((v * 1000.0) + 1000);
            else
                dutyCycle = 0;
        } else {
            if (v > 0.5) {
                GPIO_SET = 1 << conf.pin;
            } else {
                GPIO_CLR = 1 << conf.pin;
            }
        }
    }

    buttons_pi::buttons_pi()
    {
        static int configured = configure_pi();
        if (configured < 0)
            throw "could not setup raspberry pi gpio";
    }
    int buttons_pi::configure(const ButtonsPiConfig& c)
    {
        this->config = c;
        INP_GPIO(c.x);
        INP_GPIO(c.y);
        INP_GPIO(c.z);
        INP_GPIO(c.t);
        //	std::cout << "GPIO_PULL" << ((unsigned long)GPIO_PULL) << std::endl;
        //	GPIO_PULL = (GPIO_PULL) | (1 << c.x) | (1 << c.y) | (1 << c.z) | (1 << c.t);
        std::cout << "button pulls:  " << c.x << " " << c.y << " " << c.z << " " << c.t << std::endl;

        // enable pull-up on GPIO24&25
        GPIO_PULL = 2;
        volatile int cnt = 0;
        for (int i = 0; i < 1500; i++) {
            cnt += i;
        }
        GPIO_PULLCLK0 = (1 << c.x) | (1 << c.y) | (1 << c.z) | (1 << c.t);
        cnt = 0;
        for (int i = 0; i < 1500; i++) {
            cnt += i;
        }

        GPIO_PULL = 0;
        GPIO_PULLCLK0 = 0;

        for (int i = 0; i < 1500; i++) {
            cnt += i;
        }
        return cnt & 0;
    }
    std::array<unsigned char, 4> buttons_pi::getButtons()
    {
        return {
            (unsigned char)(1 - ((GPIO_READ(config.x)) >> (config.x))),
            (unsigned char)(1 - ((GPIO_READ(config.y)) >> (config.y))),
            (unsigned char)(1 - ((GPIO_READ(config.z)) >> (config.z))),
            (unsigned char)(1 - ((GPIO_READ(config.t)) >> (config.t)))
        };
    }

    p_Stepper StepperPi_factory(const std::vector<StepperPiConfig> stc)
    {
        stepper_pi* a = new stepper_pi();
        p_Stepper ret(a);
        a->configure(stc);

        return ret;
    }

    p_Spindle SpindlePi_factory(const SpindlePiConfig stc)
    {
        spindle_pi* a = new spindle_pi();
        p_Spindle ret(a);
        a->configure(stc);
        return ret;
    }

    p_Buttons ButtonsPi_factory(const ButtonsPiConfig stc)
    {
        buttons_pi* a = new buttons_pi();
        p_Buttons ret(a);
        a->configure(stc);
        return ret;
    }

    void to_json(nlohmann::json& j, const SpindlePiConfig& p)
    {
        j = nlohmann::json{
            { "pin", p.pin }, //": 18,
            { "servopwm", p.servopwm } //":1
        };
    }

    void to_json(nlohmann::json& j, const StepperPiConfig& p)
    {
        j = nlohmann::json{
            { "dir", p.dir }, //: 27,
            { "en", p.en }, //: 10,
            { "step", p.step } //: 22
        };
    }

    void from_json(const nlohmann::json& j, SpindlePiConfig& p)
    {
        try {
            p.pin = j.at("pin").get<int>();
        } catch (...) {
        }; // 18
        try {
            p.servopwm = j.at("servopwm").get<int>();
        } catch (...) {
        }; // 1
    }

    void from_json(const nlohmann::json& j, StepperPiConfig& p)
    {
        try {
            p.dir = j.at("dir").get<double>();
        } catch (...) {
        }; // 27
        try {
            p.en = j.at("en").get<double>();
        } catch (...) {
        }; // 10
        try {
            p.step = j.at("step").get<double>();
        } catch (...) {
        }; // 22
    }

    void to_json(nlohmann::json& j, const ButtonsPiConfig& p)
    {
        j = nlohmann::json{
            { "x", p.x },
            { "y", p.y },
            { "z", p.z },
            { "t", p.t },
        };
    }

    void from_json(const nlohmann::json& j, ButtonsPiConfig& p)
    {
        try {
            p.x = j.at("x").get<int>();
        } catch (...) {
        };
        try {
            p.y = j.at("y").get<int>();
        } catch (...) {
        };
        try {
            p.z = j.at("z").get<int>();
        } catch (...) {
        };
        try {
            p.t = j.at("t").get<int>();
        } catch (...) {
        };
    }

    /*
stepper_pi::stepper_pi() {}
int stepper_pi::configure( const std::vector  <StepperPiConfig> &conf ) {	this->conf = conf;	return 0;}
void stepper_pi::step( std::array<signed char, 3> &dirs ) {}
void stepper_pi::enabled( bool en ) {}
spindle_pi::spindle_pi() {}
int spindle_pi::configure( const SpindlePiConfig &c ) {	this->conf = conf;	return 0;}
void spindle_pi::setSpeed( double v ) {}





*/

} // namespace motor
} // namespace tp
