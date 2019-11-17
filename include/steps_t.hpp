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



#ifndef __RASPIGCD_STEPS_T_HPP__
#define __RASPIGCD_STEPS_T_HPP__

#include <array>
#include <iostream>

#include <distance_t.hpp>
#include <hardware_dof_conf.hpp>

/**
 * main namespace for the project raspigcd
 * */
namespace raspigcd {
using steps_t = generic_position_t<int,4>;
} // namespace raspigcd
#endif
