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



#include "m_hwmoves.hpp"
#include "coordsystem_corexy.hpp"
#include "coordsystem_euclidean.hpp"
#include "coordsystem.hpp"

#include <stdexcept>
#include <vector>

namespace tp {
namespace coord {


CoordTranslateConfig getDefaults_CoordTranslateConfig() {
	CoordTranslateConfig ret;
	ret.motorConfiguration   = "corexy";
	ret.scale.x              = 1.0;
	ret.scale.y              = 1.0;
	ret.scale.z              = 1.0;
	ret.scale.t              = 1.0;
	ret.stepsPerMm.a         = 100; //82.0512820513;
	ret.stepsPerMm.b         = 100; //82.0512820513;
	ret.stepsPerMm.c         = 100; //200;
	ret.stepsPerMm.d         = 100; //200;
	return ret;
}

void to_json( nlohmann::json& j, const CoordTranslateConfig& p ) {
	j = nlohmann::json {

		{"motorConfiguration", p.motorConfiguration}, //": "corexy",
		{
			"scale", {
				{"x", p.scale.x}, //: -1.0,
				{"y", p.scale.y}, //: 1.0,
				{"z", p.scale.z}, //: 1.0,
				{"t", p.scale.t} //: 1.0
			}
		},
		{
			"stepsPerMm", {
				{"a", p.stepsPerMm.a}, //: 82.0512820513,
				{"b", p.stepsPerMm.b}, //: 82.0512820513,
				{"c", p.stepsPerMm.c}, //: 200
				{"d", p.stepsPerMm.d} //: 200
			}
		}

	};
}

void from_json( const nlohmann::json& j, CoordTranslateConfig& p ) {
	p = getDefaults_CoordTranslateConfig();
	try { p.motorConfiguration   = j["motorConfiguration"].get<std::string>(); } catch (...) {}       ; // corexy
	try { p.scale.x              = j["scale"]["x"].get<double>();              } catch (...) {}       ; // -1.0
	try { p.scale.y              = j["scale"]["y"].get<double>();              } catch (...) {}       ; // 1.0
	try { p.scale.z              = j["scale"]["z"].get<double>();              } catch (...) {}       ; // 1.0
	try { p.scale.t              = j["scale"]["t"].get<double>();              } catch (...) {}       ; // 1.0
	try { p.stepsPerMm.a         = j["stepsPerMm"]["a"].get<double>();         } catch (...) {}       ; // 82.0512820513
	try { p.stepsPerMm.b         = j["stepsPerMm"]["b"].get<double>();         } catch (...) {}       ; // 82.0512820513
	try { p.stepsPerMm.c         = j["stepsPerMm"]["c"].get<double>();         } catch (...) {}       ; // 200
	try { p.stepsPerMm.d         = j["stepsPerMm"]["d"].get<double>();         } catch (...) {}       ; // 200
}

std::ostream& operator << ( std::ostream& os, CoordTranslateConfig const& value ) {
	nlohmann::json j;
	j = value;
	os << j.dump();
	return os;
}

std::istream& operator >> ( std::istream& is, CoordTranslateConfig & value ) {
	nlohmann::json j;
	is >> j;
	value = j;
	return is;
}

bool operator == ( const CoordTranslateConfig & l, const CoordTranslateConfig & r ) {
	if ( l.motorConfiguration   != r.motorConfiguration   ) return false;
	if ( l.scale.x              != r.scale.x              ) return false;
	if ( l.scale.y              != r.scale.y              ) return false;
	if ( l.scale.z              != r.scale.z              ) return false;
	if ( l.scale.t              != r.scale.t              ) return false;
	if ( l.stepsPerMm.a         != r.stepsPerMm.a         ) return false;
	if ( l.stepsPerMm.b         != r.stepsPerMm.b         ) return false;
	if ( l.stepsPerMm.c         != r.stepsPerMm.c         ) return false;
	if ( l.stepsPerMm.d         != r.stepsPerMm.d         ) return false;
	return true;
}







CoordTranslateSimple::CoordTranslateSimple( const std::array<double,4> stepsPerMM_, const Position scaleAxis ) {
	//if ( stepsPerMm_.size() < 4 ) throw "4 axis must be";
	if((mX = stepsPerMM_[0]) == 0) throw std::invalid_argument("mX must not be 0");
	if((mY = stepsPerMM_[1]) == 0) throw std::invalid_argument("mY must not be 0");
	if((mZ = stepsPerMM_[2]) == 0) throw std::invalid_argument("mZ must not be 0");
	if((mT = stepsPerMM_[3]) == 0) throw std::invalid_argument("mT must not be 0");
	if((sX = scaleAxis[0]) == 0) throw std::invalid_argument("sX must not be 0");
	if((sY = scaleAxis[1]) == 0) throw std::invalid_argument("sY must not be 0");
	if((sZ = scaleAxis[2]) == 0) throw std::invalid_argument("sZ must not be 0");
	if((sT = scaleAxis[3]) == 0) throw std::invalid_argument("sT must not be 0");
}
Steps CoordTranslateSimple::translate( const Position &pos ) {
	return Steps( pos[0] * mX * sX, pos[1] * mY * sY, pos[2] * mZ * sZ, pos[3] * mT * sT );
};
Position CoordTranslateSimple::translate( const Steps &steps ) {
	return Position( ( double )steps[0] / ( sX * mX ), ( double )steps[1] / ( sY * mY ), ( double )steps[2] / ( sZ * mZ ), ( double )steps[3] / ( sT * mT ) );
};


CoordTranslateCoreXY::CoordTranslateCoreXY( const std::array<double,4> stepsPerMM_, const Position scaleAxis ) {
	//if (stepsPerMm_.size() < 4) throw "4 axis must be";
	if((mX = stepsPerMM_[0]) == 0) throw std::invalid_argument("mX must not be 0");
	if((mY = stepsPerMM_[1]) == 0) throw std::invalid_argument("mY must not be 0");
	if((mZ = stepsPerMM_[2]) == 0) throw std::invalid_argument("mZ must not be 0");
	if((mT = stepsPerMM_[3]) == 0) throw std::invalid_argument("mT must not be 0");
	if((sX = scaleAxis[0]) == 0) throw std::invalid_argument("sX must not be 0");
	if((sY = scaleAxis[1]) == 0) throw std::invalid_argument("sY must not be 0");
	if((sZ = scaleAxis[2]) == 0) throw std::invalid_argument("sZ must not be 0");
	if((sT = scaleAxis[3]) == 0) throw std::invalid_argument("sT must not be 0");
	
}

Steps CoordTranslateCoreXY::translate( const Position &pos ) {
	return Steps( ( pos[0] * sX + pos[1] * sY ) * mX, ( pos[0] * sX - pos[1] * sY ) * mY, pos[2] * mZ * sZ , pos[3] * mT * sT);
};
Position CoordTranslateCoreXY::translate( const Steps &steps ) {
	return Position( 0.5 * ( double )( steps[0] / mX + steps[1] / mY ) / sX, 0.5 * ( double )( steps[0] / mX - steps[1] / mY ) / sY,
	 steps[2] / ( mZ * sZ ), steps[3] / ( mT * sT ) );
};


std::shared_ptr < i_CoordTranslate > CoordTranslate_corexy_factory( const std::array<double,4> stepsPerMM_, const Position scaleAxis ) {
	return std::shared_ptr < i_CoordTranslate >( new CoordTranslateCoreXY( stepsPerMM_, scaleAxis ) );
}
std::shared_ptr < i_CoordTranslate >  CoordTranslate_simple_factory( const std::array<double,4> stepsPerMM_, const Position scaleAxis ) {
	return std::shared_ptr < i_CoordTranslate >( new CoordTranslateSimple( stepsPerMM_, scaleAxis ) );
}


std::shared_ptr < i_CoordTranslate >  CoordTranslate_factory( const CoordTranslateConfig &config ) {
	if ( config.motorConfiguration == "corexy" ) {
		return CoordTranslate_corexy_factory( {config.stepsPerMm.a, config.stepsPerMm.b, config.stepsPerMm.c,  config.stepsPerMm.d},
			{config.scale.x, config.scale.y, config.scale.z, config.scale.t} );
	}
	if ( config.motorConfiguration == "simple" ) {
		return CoordTranslate_simple_factory(  {config.stepsPerMm.a, config.stepsPerMm.b, config.stepsPerMm.c, config.stepsPerMm.d}, {
			config.scale.x, config.scale.y, config.scale.z, config.scale.t} );
	}
	throw std::domain_error( "config.motorConfiguration can be only corexy or simple" );
}


}
}






