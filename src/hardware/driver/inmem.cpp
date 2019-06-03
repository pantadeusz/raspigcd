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
#include <distance_t.hpp>
#include <hardware/driver/inmem.hpp>
#include <hardware/low_buttons.hpp>
#include <hardware/low_spindles_pwm.hpp>
#include <hardware/low_steppers.hpp>
#include <hardware/stepping_commands.hpp>
#include <steps_t.hpp>

#include <functional>
#include <map>
#include <memory>
#include <thread>

namespace raspigcd {
namespace hardware {
namespace driver {

void inmem::set_step_callback(std::function<void(const steps_t&)> on_step_) {
    _on_step = on_step_;
}
void inmem::do_step(const std::array<single_step_command,4> &b)
{
    for (size_t i = 0; i < counters.size(); i++) {
        if (b[i].step == 1) {
            counters[i] += b[i].dir * 2 - 1;
        }
    }
    for (size_t j = 0; j < current_steps.size(); j++)
        current_steps[j] = current_steps[j] + (int)((signed char)b[j].step * ((signed char)b[j].dir * 2 - 1));
    _on_step(current_steps);
};

void inmem::enable_steppers(const std::vector<bool> en)
{
    enabled = en;
    on_enable_steppers(en);
};


inmem::inmem()
{
    current_steps = {0,0,0};
    for (int i = 0; i < counters.size(); i++) {
        counters[i] = 0;
    }
    enabled = std::vector<bool>(false, counters.size());
    _on_step = [](const steps_t&) {};

    on_enable_steppers = [](const std::vector<bool>){};
}
} // namespace driver
} // namespace hardware
} // namespace raspigcd

