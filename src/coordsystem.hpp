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


#ifndef __COORDSYSTEM__HPP___
#define  __COORDSYSTEM__HPP___


#include "m_hwmoves.hpp"
#include <tpcommon/position.hpp>

#include <nlohmann/json.hpp>

#include <vector>

namespace tp {
namespace coord {

using tp::motor::Steps;


struct CoordTranslateConfig {
	std::string motorConfiguration; //: "corexy",
	struct  {
		double x;//: -1.0,
		double y;//: 1.0,
		double z;//: 1.0
		double t;//: 1.0
	} scale;
	struct  {
		double a;//: 82.0512820513,
		double b;//: 82.0512820513,
		double c;//: 200
		double d;//: 100
	} stepsPerMm;
};


CoordTranslateConfig getDefaults_CoordTranslateConfig();
void to_json( nlohmann::json& j, const CoordTranslateConfig& p ) ;
void from_json( const nlohmann::json& j, CoordTranslateConfig& p );
std::ostream& operator << ( std::ostream& os, CoordTranslateConfig const& value );
std::istream& operator >> ( std::istream& is, CoordTranslateConfig & value );
bool operator == ( const CoordTranslateConfig & l, const CoordTranslateConfig & r );




class i_CoordTranslate  {
public:
	virtual Steps translate( const Position &pos ) = 0;
	virtual Position translate( const Steps &steps ) = 0;
};


std::shared_ptr < i_CoordTranslate >  CoordTranslate_corexy_factory( const std::vector<double> stepsPerMM_, const Position scaleAxis = Position( 1, 1, 1 ) );
std::shared_ptr < i_CoordTranslate >  CoordTranslate_simple_factory( const std::vector<double> stepsPerMM_, const Position scaleAxis = Position( 1, 1, 1 ) );
std::shared_ptr < i_CoordTranslate >  CoordTranslate_factory( const CoordTranslateConfig &config );


}
}



#endif




