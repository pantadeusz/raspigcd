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


#ifndef __M_MOTOR_RASPBERRY_PI_HPP___
#define __M_MOTOR_RASPBERRY_PI_HPP___ 1

#include "m_motor_interface.hpp"
#include <nlohmann/json.hpp>

namespace tp {
namespace motor {

struct StepperPiConfig {
	unsigned int step;
	unsigned int dir;
	unsigned int en;
};
struct SpindlePiConfig {
	unsigned int pin;
	int servopwm;
};
struct ButtonsPiConfig {
	unsigned int x;
	unsigned int y;
	unsigned int z;
	unsigned int t;
};

void to_json( nlohmann::json& j, const StepperPiConfig& p );
void from_json( const nlohmann::json& j, StepperPiConfig& p );
void to_json( nlohmann::json& j, const SpindlePiConfig& p );
void from_json( const nlohmann::json& j, SpindlePiConfig& p );
void to_json( nlohmann::json& j, const ButtonsPiConfig& p );
void from_json( const nlohmann::json& j, ButtonsPiConfig& p );


inline bool operator == ( const StepperPiConfig &l, const StepperPiConfig &r ) {
	return ( l.step==r.step ) && ( l.dir == r.dir ) && ( l.en == r.en );
};

inline std::ostream& operator << ( std::ostream& os, StepperPiConfig const& value ) {
	os << value.step << ", " << value.dir << ", " <<  value.en;
	return os;
}


p_Stepper StepperPi_factory( const std::vector < StepperPiConfig > stc );
p_Spindle SpindlePi_factory( const  SpindlePiConfig stc );
p_Buttons ButtonsPi_factory( const  ButtonsPiConfig stc );
// w_Stepper wStepperPi_factory(const StepperPiConfig stc);

}
}

#endif



