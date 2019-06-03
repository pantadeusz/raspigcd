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

#ifndef __TP_COORD_I_COORDTRANSLATE_HPP___
#define __TP_COORD_I_COORDTRANSLATE_HPP___

#include "dto_CoordTranslateConfig.hpp"
#include "dto_Steps.hpp"
#include <tpcommon/position.hpp>

#include <nlohmann/json.hpp>

#include <array>

namespace tp {
namespace coord {

    class i_CoordTranslate {
    public:
        virtual tp::motor::Steps translate(const Position& pos) = 0;
        virtual Position translate(const tp::motor::Steps& steps) = 0;
    };

} // namespace coord
} // namespace tp

#endif
