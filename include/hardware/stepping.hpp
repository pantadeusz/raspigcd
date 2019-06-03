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


#ifndef __RASPIGCD_HARDWARE_STEPPING_T_HPP__
#define __RASPIGCD_HARDWARE_STEPPING_T_HPP__

#include <atomic>
#include <configuration.hpp>
#include <distance_t.hpp>
#include <functional>
#include <hardware/low_steppers.hpp>
#include <hardware/low_timers.hpp>
#include <hardware/stepping_commands.hpp>
#include <memory>
#include <steps_t.hpp>
#include <list>

namespace raspigcd {
namespace hardware {

//raspigcd::hardware::execution_terminated

class execution_terminated : public std::exception {
public:
    const steps_t delta_steps;
    execution_terminated (const steps_t ds = steps_t()):delta_steps(ds){}

    const char* what() const noexcept
    {
        static const char what_[]="stepping has been terminated";
        return what_;
    }
};

/**
 * @brief The basic class that allows for generating steps on the machine with precise timing
 */

class stepping
{
public:
    /**
	* @brief Executes multistep commands list. On each step it calls on_step_
     function with current position as an argument. The return value is the final steps value
	*
	* @param start_steps the initial position of motors in steps
	* @param commands_to_do array of commands to execute
	* // @param on_step_ function to execute on each step. This function can throw exceptions to break execution of steps
	* @return steps_t final position of the machine in steps
	*/
    virtual void exec(const multistep_commands_t& commands_to_do,
    std::function<int (const steps_t steps_from_start, const int command_index) > on_execution_break = [](auto,auto){return 0;}) = 0;
    /**
     * returns current tick index. This is not in the terms of commands. There will be at least as many ticks as commands.#pragma endregion
     * */
    virtual int get_tick_index() const = 0;

    /**
     * breaks execution of the exec method.
     * if n > 0 that means it shlud stop in n steps
     * */
    virtual void terminate(const int n = 0) = 0;

    virtual void reset_after_terminate() = 0;
};

class stepping_sim : public stepping
{
    std::shared_ptr<low_steppers> _steppers_driver_shr;
    std::function<void(const steps_t&)> _on_step;

    std::atomic<int> _terminate_execution;
public:
    std::atomic<int> _tick_index;

    steps_t current_steps;

    void set_callback(std::function<void(const steps_t&)> on_step_ = [](const steps_t&) {}) {
        _on_step = on_step_;
    }

    virtual int get_tick_index() const {return _tick_index;};
// const steps_t& start_steps, std::function<void(const steps_t&)> on_step_
    void exec(const multistep_commands_t& commands_to_do,
    std::function<int (const steps_t steps_from_start, const int command_index) > on_execution_break = [](auto,auto){return 0;});

    void terminate(const int n= 0) {
        _terminate_execution = 1+n;
    }
    void reset_after_terminate() {_terminate_execution = 0;};    

    stepping_sim(
        const steps_t start_steps_,
        std::function<void(const steps_t&)> on_step_ = [](const steps_t&) {}) {
        current_steps = start_steps_;
        set_callback(on_step_);
    }
};

class stepping_simple_timer : public stepping
{
    std::atomic<int> _steps_counter; 
    std::atomic<int> _tick_index; 
    std::atomic<int> _terminate_execution;

public:
/**
 * @brief returns the counter that is incremented whenever stepper motor performs step
 * 
 */
    virtual std::atomic<int> *get_step_counter() {
        return &_steps_counter;
    };
    virtual int get_tick_index() const {return _tick_index;};

    std::atomic<int> _delay_microseconds;
    std::shared_ptr<low_steppers> _steppers_driver_shr;
    low_steppers* _steppers_driver;

    std::shared_ptr<low_timers> _low_timer_shr;
    low_timers *_low_timer;

    /**
     * @brief Set the delay in microseconds
     * 
     * @param delay_us the intended minimal step delay
     */
    void set_delay_microseconds(int delay_us);

    /**
     * @brief Set the low level steppers driver
     * 
     * @param steppers_driver pointer to the steppers driver. This can be faked or true
     */
    void set_low_level_steppers_driver(std::shared_ptr<low_steppers> steppers_driver);

    void set_low_level_timers(std::shared_ptr<low_timers> timer_drv_);

    void exec(const multistep_commands_t& commands_to_do,
    std::function<int (const steps_t steps_from_start, const int command_index) > on_execution_break = [](auto,auto){return 0;});

    void terminate(const int n = 0) {
        if (_terminate_execution == 0) _terminate_execution = 1+n;
    }
    void reset_after_terminate() {_terminate_execution = 0;};


    stepping_simple_timer(int delay_us, std::shared_ptr<low_steppers> steppers_driver, std::shared_ptr<low_timers> timer_drv_)
    {
        set_delay_microseconds(delay_us);
        set_low_level_steppers_driver(steppers_driver);
        set_low_level_timers(timer_drv_);
    }

    stepping_simple_timer(const configuration::global& conf, std::shared_ptr<low_steppers> steppers_driver, std::shared_ptr<low_timers> timer_drv_)
    {
        set_delay_microseconds(conf.tick_duration_us);
        set_low_level_steppers_driver(steppers_driver);
        set_low_level_timers(timer_drv_);
    }
};




std::list<steps_t> hardware_commands_to_steps(const multistep_commands_t& commands_to_do);
/**
 * @brief Calculates position after execution of given number of steps
 * 
 * @param commands_to_do list of commands to execute
 * @param last_step_ the break after given number of steps. If 0, then no steps are performed.
 */
steps_t hardware_commands_to_last_position_after_given_steps(const std::vector<multistep_command>& commands_to_do, int last_step_ = -1);
// untested:
int hardware_commands_to_steps_count(const std::vector<multistep_command>& commands_to_do, int last_step_ = -1);

} // namespace hardware
} // namespace raspigcd

#endif
