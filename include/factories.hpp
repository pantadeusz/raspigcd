
#ifndef __RASPIGCD_FACTORIES_T_HPP__
#define __RASPIGCD_FACTORIES_T_HPP__

#include <hardware/driver/inmem.hpp>
#include <hardware/driver/low_buttons_fake.hpp>
#include <hardware/driver/low_spindles_pwm_fake.hpp>
#include <hardware/driver/low_timers_busy_wait.hpp>
#include <hardware/driver/low_timers_fake.hpp>
#include <hardware/driver/low_timers_wait_for.hpp>
#include <hardware/driver/raspberry_pi.hpp>
#include <hardware/motor_layout.hpp>
#include <hardware/stepping.hpp>


#include <map>
#include <memory>
#include <string>
#include <vector>


namespace raspigcd {

std::tuple<
    std::shared_ptr<raspigcd::hardware::low_timers>,
    std::shared_ptr<raspigcd::hardware::low_steppers>,
    std::shared_ptr<raspigcd::hardware::low_spindles_pwm>,
    std::shared_ptr<raspigcd::hardware::low_buttons>,
    std::shared_ptr<raspigcd::hardware::motor_layout>,
    std::shared_ptr<raspigcd::hardware::stepping_simple_timer>>
stepping_simple_timer_factory(configuration::global cfg);

}

#endif