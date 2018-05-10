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
#define  __COORDSYSTEM_COREXY_HPP___

#include "m_hwmoves.hpp"
#include "coordsystem.hpp"
#include <vector>

namespace tp {
namespace coord {

using tp::motor::Steps;

class CoordTranslateCoreXY : public i_CoordTranslate {
protected:
	double mX, mY, mZ;
	double sX, sY, sZ;
public:
	CoordTranslateCoreXY( const double stepsPerMM_x_, const double stepsPerMM_y_, const double stepsPerMM_z_, const Position scaleAxis );
	Steps translate( const Position &pos );
	Position translate( const Steps &steps );
};


}
}



#endif




