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

#ifndef __COORDSYSTEM_COREXY_HPP___
#define __COORDSYSTEM_COREXY_HPP___

#include "coordsystem.hpp"
#include "m_hwmoves.hpp"
#include <vector>

namespace tp {
namespace coord {

    using tp::motor::Steps;

    class CoordTranslateCoreXY : public i_CoordTranslate {
    protected:
        double mX, mY, mZ, mT;
        double sX, sY, sZ, sT;

    public:
        CoordTranslateCoreXY(const std::array<double, 4> stepsPerMM_, const Position scaleAxis);
        Steps translate(const Position& pos);
        Position translate(const Steps& steps);
    };

} // namespace coord
} // namespace tp

#endif
