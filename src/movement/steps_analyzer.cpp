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




#include <distance_t.hpp>
#include <hardware/stepping_commands.hpp>
#include <list>
#include <movement/simple_steps.hpp>
#include <movement/steps_analyzer.hpp>
#include <steps_t.hpp>

namespace raspigcd {
namespace movement {

steps_t steps_analyzer::steps_from_tick(const hardware::multistep_commands_t& commands_to_do, const int tick_number) const
{
    auto tick_number_ = tick_number;
    steps_t _steps = {0, 0, 0, 0};
    int cmnd_i = 0;
    int i = 0;
    for (const auto& s : commands_to_do) {
        //std::cout << " i " << i << " cmnd_i " << cmnd_i << std::endl;
        if ((tick_number_ >= i) && (tick_number_ < (i + s.count))) {
            //std::cout << " i " << i << " cmnd_i " << cmnd_i << "  >> " << (tick_number_ - i) << std::endl;
            for (int j = 0; j < 4; j++)
                _steps[j] = _steps[j] + (tick_number_ - i) * ((int)((signed char)s.b[j].step * ((signed char)s.b[j].dir * 2 - 1)));
            return _steps;
        } else {
            for (int j = 0; j < 4; j++)
                _steps[j] = _steps[j] + s.count * ((int)((signed char)s.b[j].step * ((signed char)s.b[j].dir * 2 - 1)));
        }
        cmnd_i++;
        i += s.count;
    }
    if (i == tick_number_)
        return _steps;
    throw std::out_of_range("the index is after the last step");
};

int steps_analyzer::get_last_tick_index(const hardware::multistep_commands_t& commands_to_do) const
{
    int cmnd_i = 0;
    int i = 0;
    for (const auto& s : commands_to_do) {
        cmnd_i++;
        i += s.count;
    }
    return i;
};

} // namespace movement
} // namespace raspigcd
