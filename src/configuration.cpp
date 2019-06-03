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
#include <fstream>
#include <iostream>
#include <json/json.hpp>

#include <distance_t.hpp>

namespace raspigcd {
namespace configuration {

double limits::proportional_max_accelerations_mm_s2(const distance_t& norm_vect) const {
    return calculate_linear_coefficient_from_limits(max_accelerations_mm_s2, norm_vect);
}
double limits::proportional_max_velocity_mm_s(const distance_t& norm_vect) const {
    return calculate_linear_coefficient_from_limits(max_velocity_mm_s, norm_vect);
}
double limits::proportional_max_no_accel_velocity_mm_s(const distance_t& norm_vect) const {
    return calculate_linear_coefficient_from_limits(max_no_accel_velocity_mm_s, norm_vect);
}


double global::tick_duration() const
{ // czas ticku w sekundach. 0.00005 = 50mikrosekund
    return ((double) tick_duration_us)/1000000.0; // 0.000001 * (double)tick_duration_us;
}

global& global::load_defaults()
{
    tick_duration_us = 50;
    simulate_execution = false;

    douglas_peucker_marigin = 1.0/64.0;

    motion_layout = COREXY; //"corexy";
    lowleveltimer = BUSY_WAIT;
    scale = {1.0, 1.0, 1.0};
    max_accelerations_mm_s2 = {200.0, 200.0, 200.0};
    max_velocity_mm_s = {220.0, 220.0, 110.0};  ///<maximal velocity on axis in mm/s
    max_no_accel_velocity_mm_s = {2.0, 2.0, 2.0}; ///<maximal velocity on axis in mm/s

    steppers = {
        stepper(27, 10, 22, 100.0),
        stepper(4, 10, 17, 100.0),
        stepper(9, 10, 11, 100.0)
        //,
        //stepper(0, 10, 5, 100.0)
        };
    buttons = {
        {.pin = 21, .pullup = true}, 
        {.pin = 20, .pullup = true}, 
        {.pin = 16, .pullup = true}, 
        {.pin = 12, .pullup = true}};
    spindles = {
        {.pin = 18,
            .cycle_time_seconds = 0.1,
            .duty_min = 0.0,
            .duty_max = 0.1}
        /*,        {
            pin : 18,
            cycle_time_seconds : 0.02,
            duty_min : 0.001,
            duty_max : 0.002   // 20ms
        }
*/
    };

    return *this;
}

global& global::load(const std::string& filename)
{
    std::ifstream ifs(filename);
    (*this) = nlohmann::json::parse(ifs);
    return *this;
}

global& global::save(const std::string& filename)
{
    std::ofstream file(filename);
    file << (*this);
    return *this;
}


/* **************************************************************************
 * CONVERSIONS
 * ************************************************************************** */

void to_json(nlohmann::json& j, const spindle_pwm& p)
{
    j = nlohmann::json{
        {"pin", p.pin},
        {"cycle_time_seconds", p.cycle_time_seconds}, // 20ms
        {"duty_min", p.duty_min},
        {"duty_max", p.duty_max}};
}

void from_json(const nlohmann::json& j, spindle_pwm& p)
{
    p.pin = j.value("pin", p.pin);
    p.cycle_time_seconds = j.value("cycle_time_seconds", p.cycle_time_seconds);
    p.duty_min = j.value("duty_min", p.duty_min);
    p.duty_max = j.value("duty_max", p.duty_max);
}

std::ostream& operator<<(std::ostream& os, spindle_pwm const& value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}


void to_json(nlohmann::json& j, const stepper& p)
{
    j = nlohmann::json{
        {"step", p.step},
        {"dir", p.dir},
        {"en", p.en},
        {"steps_per_mm", p.steps_per_mm}};
}

void from_json(const nlohmann::json& j, stepper& p)
{
    p.step = j.value("step", p.step);
    p.dir = j.value("dir", p.dir);
    p.en = j.value("en", p.en);
    p.steps_per_mm = j.value("steps_per_mm", p.steps_per_mm);
    p.steps_per_mm = j.value("steps_per_m", p.steps_per_m()) / 1000.0;
    if (p.steps_per_mm <= 1.0)
        throw std::invalid_argument("the steps_per_mm must be greater than 1.0");
}

std::ostream& operator<<(std::ostream& os, stepper const& value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}

void to_json(nlohmann::json& j, const button& p)
{
    j = nlohmann::json{
        {"pin", p.pin},
        {"pullup", p.pullup}};
}

void from_json(const nlohmann::json& j, button& p)
{
    p.pin = j.value("pin", p.pin);
    p.pullup = j.value("pullup", p.pullup);
}


auto lowleveltimertostring = [](auto llt){
    return (llt == BUSY_WAIT) ? 
            "low_timers_busy_wait" : (
               (llt == WAIT_FOR) ? "low_timers_wait_for" : "low_timers_fake"
            );
};

void to_json(nlohmann::json& j, const global& p)
{
    j = nlohmann::json{
        {"tick_duration_us", p.tick_duration_us},
        {"simulate_execution", p.simulate_execution},
        {"douglas_peucker_marigin", p.douglas_peucker_marigin},
        {"lowleveltimer", lowleveltimertostring(p.lowleveltimer)},
        {"motion_layout", (p.motion_layout == COREXY) ? "corexy" : "cartesian"},
        {"scale", p.scale},
        {"max_accelerations_mm_s2", p.max_accelerations_mm_s2},
        {"max_velocity_mm_s", p.max_velocity_mm_s},
        {"max_no_accel_velocity_mm_s", p.max_no_accel_velocity_mm_s},
        {"spindles", p.spindles},
        {"steppers", p.steppers},
        {"lasers", p.lasers},
        {"buttons", p.buttons}};
}

void from_json(const nlohmann::json& j, global& p)
{
    p.simulate_execution = j.value("simulate_execution", p.simulate_execution);
    p.douglas_peucker_marigin = j.value("douglas_peucker_marigin", p.douglas_peucker_marigin);
    p.tick_duration_us = j.value("tick_duration_us", p.tick_duration_us);

    {
        //p.lowleveltimer = j.value("lowleveltimer", p.lowleveltimer);
        std::string s = j.value("lowleveltimer", lowleveltimertostring(p.lowleveltimer));
        if (!((s == "low_timers_busy_wait") || 
        (s == "low_timers_wait_for") || 
        (s == "low_timers_fake")
        )) throw std::invalid_argument("lowleveltimer can be only low_timers_busy_wait or low_timers_wait_for or low_timers_fake");
        p.lowleveltimer = (s == "low_timers_busy_wait") ? BUSY_WAIT : p.lowleveltimer;
        p.lowleveltimer = (s == "low_timers_wait_for") ? WAIT_FOR : p.lowleveltimer;
        p.lowleveltimer = (s == "low_timers_fake") ? FAKE : p.lowleveltimer;
    }

    {
        std::string s = j.value("motion_layout", (p.motion_layout == COREXY) ? "corexy" : "cartesian");
    if (!((s == "corexy") || (s == "cartesian"))) throw std::invalid_argument("motion_layout can be only corexy or cartesian");
    p.motion_layout = (s == "corexy") ? COREXY : p.motion_layout;
    p.motion_layout = (s == "cartesian") ? CARTESIAN : p.motion_layout;
    }

    //(std::array<double,4>)
//    std::cout << "have: " << p.scale << " -> " << j["scale"] << " -> ";
    std::vector<double> tmp = j.value("scale", std::vector<double>(p.scale.begin(),p.scale.end()));
    p.scale = tmp;//(std::array<double,4>)p.scale);
//    for (auto e : tmp) std::cout << e << ",";
//    std::cout << " -> " << p.scale << std::endl;
    tmp = j.value("max_accelerations_mm_s2", std::vector<double>(p.max_accelerations_mm_s2.begin(),p.max_accelerations_mm_s2.end())); p.max_accelerations_mm_s2 = tmp;
    tmp = j.value("max_velocity_mm_s", std::vector<double>(p.max_velocity_mm_s.begin(),p.max_velocity_mm_s.end())); p.max_velocity_mm_s = tmp;
    tmp = j.value("max_no_accel_velocity_mm_s", std::vector<double>(p.max_no_accel_velocity_mm_s.begin(),p.max_no_accel_velocity_mm_s.end())); p.max_no_accel_velocity_mm_s = tmp;

    p.spindles = j.value("spindles", p.spindles);
    p.steppers = j.value("steppers", p.steppers);
    p.buttons = j.value("buttons", p.buttons);
    p.lasers = j.value("lasers", p.lasers);
}

std::ostream& operator<<(std::ostream& os, global const& value)
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}

bool operator==(const global& l, const global& r)
{
    return (l.tick_duration_us == r.tick_duration_us) &&
           (l.max_accelerations_mm_s2 == r.max_accelerations_mm_s2) &&
           (l.max_velocity_mm_s == r.max_velocity_mm_s) &&
           (l.max_no_accel_velocity_mm_s == r.max_no_accel_velocity_mm_s) &&
           (l.scale == r.scale) &&
           (l.motion_layout == r.motion_layout) &&
           (l.spindles == r.spindles) &&
           (l.steppers == r.steppers) &&
           (l.buttons == r.buttons) &&
           (l.simulate_execution == r.simulate_execution) &&
           (l.douglas_peucker_marigin == r.douglas_peucker_marigin) &&
           (l.lowleveltimer == r.lowleveltimer);
}

bool operator==(const button& l, const button& r)
{
    return (l.pin == r.pin) &&
           (l.pullup == r.pullup);
}

bool operator==(const stepper& l, const stepper& r)
{
    return (l.dir == r.dir) &&
           (l.en == r.en) &&
           (l.step == r.step) &&
           (l.steps_per_mm == r.steps_per_mm);
}
bool operator==(const spindle_pwm& l, const spindle_pwm& r)
{
    return (l.pin == r.pin) &&
           (l.cycle_time_seconds == r.cycle_time_seconds) &&
           (l.duty_min == r.duty_min) &&
           (l.duty_max == r.duty_max);
}


void to_json( nlohmann::json& j, const sync_laser& p )
{
    j = nlohmann::json{
        {"pin", p.pin},
        {"hi_is_off", p.hi_is_off}};
}
void from_json( const nlohmann::json& j, sync_laser& p )
{
    p.hi_is_off = j.value("hi_is_off", p.hi_is_off);
    p.pin = j.value("pin", p.pin);
}
std::ostream& operator<<( std::ostream& os, sync_laser const& value )
{
    nlohmann::json j = value;
    os << j.dump(2);
    return os;
}
bool operator==(const sync_laser& l, const sync_laser& r)
{
    return (l.pin == r.pin) &&
           (l.hi_is_off == r.hi_is_off);
}


} // namespace configuration
} // namespace raspigcd