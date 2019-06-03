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


#ifndef __RASPIGCD_HARDWARE_RASPBERRY_PI_3_T_HPP__
#define __RASPIGCD_HARDWARE_RASPBERRY_PI_3_T_HPP__

#include <configuration.hpp>
#include <distance_t.hpp>
#include <hardware/low_buttons.hpp>
#include <hardware/low_spindles_pwm.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/stepping_commands.hpp>
#include <hardware/low_timers.hpp>
#include <steps_t.hpp>

#include <functional>
#include <map>
#include <memory>
#include <thread>

namespace raspigcd {
namespace hardware {

namespace driver {
// IO Acces
struct bcm2835_peripheral {
    unsigned long addr_p;
    int mem_fd;
    void* map;
    volatile unsigned int* addr;
};

class raspberry_pi_3 : public low_buttons, public low_steppers, public low_spindles_pwm// , public low_timers
{
private:
    std::vector<configuration::spindle_pwm> spindles;
    std::vector<configuration::stepper> steppers;
    std::vector<configuration::button> buttons;
    std::vector<int> buttons_state;
    std::vector<std::function<void(int,int)> > buttons_callbacks;

    bool _threads_alive;
    std::vector<std::thread> _spindle_threads;
    std::vector<double> _spindle_duties;

    std::vector<bool> _enabled_steppers;

    std::thread _btn_thread;

    struct bcm2835_peripheral gpio;


public:
    /**
     * @brief attach callback to button down. It will throw exception for not supported button
     * @param callback_ the callback function that will receive button number and new status
     */
    void on_key(int btn, std::function<void(int,int)> callback_);

    /**
     * @brief returns current handler for key down
     */
    std::function<void(int,int)> on_key(int btn);
    /**
     * @brief returns the key state
     */
    virtual std::vector < int > keys_state();


    /**
	 * @brief execute single step command. That is the single most basic "step-dir" action
	 *
	 * @param b step-dir for every stepper motor (depends on the )
	 */
    void do_step(const std::array<single_step_command,4> &b);

    /**
	 * @brief turn on or off the stepper motors. If the hardware supports it, then
	 *        each motor can be enabled independently
	 *
	 * @param en enable status for each motor
	 * @return raspberry_pi_3&  returns this object
	 */
    void enable_steppers(const std::vector<bool> en);

    /**
	 * @brief Set the spindle pwm power
	 *
	 * @param i index of spindle (usualy 0)
	 * @param v value between 0 (stop) and 1 (maximal speed)
	 * @return raspberry_pi_3&  the reference to this object
	 */
    void spindle_pwm_power(const int i, const double v);

    // /**
    //  * @brief delay in seconds. This can be fraction of a second
    //  * 
    //  * @param t 
    //  */
    // void wait_s(const double t);

    /**
	 * @brief Construct a new raspberry pi 3 object
	 *
	 * @param configuration the configuration
	 */
    raspberry_pi_3(const configuration::global& configuration);

    /**
	 * @brief Destroy the raspberry pi 3 object
	 */
    virtual ~raspberry_pi_3();


    raspberry_pi_3(raspberry_pi_3 const&) = delete;
    void operator=(raspberry_pi_3 const& x) = delete;
};

} // namespace driver
} // namespace hardware
} // namespace raspigcd

#endif
