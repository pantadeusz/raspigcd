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


#include <hardware/thread_helper.hpp>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>


namespace raspigcd {
namespace hardware {

void set_thread_realtime()
{
    sched_param sch_params;
    sch_params.sched_priority = sched_get_priority_max(SCHED_RR);
    if (pthread_setschedparam(pthread_self(), SCHED_RR, &sch_params)) {
        static int already_warned = 0;
        if (already_warned == 0) {
            std::cerr << "Warning: Failed to set Thread scheduling : "
                      << std::strerror(errno) << std::endl;
            already_warned++;
        }
    }
}

} // namespace hardware
} // namespace raspigcd
