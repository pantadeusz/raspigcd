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

#include "s_CoordTranslateSimple.hpp"
#include <stdexcept>
#include <vector>

namespace tp {
namespace coord {

    CoordTranslateSimple::CoordTranslateSimple(const std::array<double, 4> stepsPerMM_, const Position scaleAxis)
    {
        //if ( stepsPerMm_.size() < 4 ) throw "4 axis must be";
        if ((mX = stepsPerMM_[0]) == 0)
            throw std::invalid_argument("mX must not be 0");
        if ((mY = stepsPerMM_[1]) == 0)
            throw std::invalid_argument("mY must not be 0");
        if ((mZ = stepsPerMM_[2]) == 0)
            throw std::invalid_argument("mZ must not be 0");
        if ((mT = stepsPerMM_[3]) == 0)
            throw std::invalid_argument("mT must not be 0");
        if ((sX = scaleAxis[0]) == 0)
            throw std::invalid_argument("sX must not be 0");
        if ((sY = scaleAxis[1]) == 0)
            throw std::invalid_argument("sY must not be 0");
        if ((sZ = scaleAxis[2]) == 0)
            throw std::invalid_argument("sZ must not be 0");
        if ((sT = scaleAxis[3]) == 0)
            throw std::invalid_argument("sT must not be 0");
    }
    Steps CoordTranslateSimple::translate(const Position& pos)
    {
        return Steps(pos[0] * mX * sX, pos[1] * mY * sY, pos[2] * mZ * sZ, pos[3] * mT * sT);
    };
    Position CoordTranslateSimple::translate(const Steps& steps)
    {
        return Position((double)steps[0] / (sX * mX), (double)steps[1] / (sY * mY), (double)steps[2] / (sZ * mZ), (double)steps[3] / (sT * mT));
    };
} // namespace coord
} // namespace tp
