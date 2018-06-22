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

#ifndef __TP_COORD_I_COORDTRANSLATE_FACTORY_HPP___
#define __TP_COORD_I_COORDTRANSLATE_FACTORY_HPP___

#include "i_CoordTranslate_factory.hpp"
#include "dto_CoordTranslateConfig.hpp"
#include "i_CoordTranslate.hpp"
#include <tpcommon/position.hpp>
#include <nlohmann/json.hpp>

#include <array>

namespace tp {
namespace coord {

    std::shared_ptr<i_CoordTranslate> CoordTranslate_corexy_factory(const std::array<double, 4> stepsPerMM_, const Position scaleAxis = Position(1, 1, 1, 1));
    std::shared_ptr<i_CoordTranslate> CoordTranslate_simple_factory(const std::array<double, 4> stepsPerMM_, const Position scaleAxis = Position(1, 1, 1, 1));
    std::shared_ptr<i_CoordTranslate> CoordTranslate_factory(const CoordTranslateConfig& config);

} // namespace coord
} // namespace tp

#endif
