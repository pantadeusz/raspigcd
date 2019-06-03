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


#include <configuration.hpp>
#include <json/json.hpp>

namespace raspigcd {
namespace configuration {

/* **************************************************************************
 * CONVERSIONS
 * ************************************************************************** */

void to_json( nlohmann::json& j, const global& p );
void from_json( const nlohmann::json& j, global& p );
std::ostream& operator<<( std::ostream& os, global const& value );

void to_json( nlohmann::json& j, const spindle_pwm& p );
void from_json( const nlohmann::json& j, spindle_pwm& p );
std::ostream& operator<<( std::ostream& os, spindle_pwm const& value );

void to_json( nlohmann::json& j, const stepper& p );
void from_json( const nlohmann::json& j, stepper& p );
std::ostream& operator<<( std::ostream& os, stepper const& value );

void to_json( nlohmann::json& j, const button& p );
void from_json( const nlohmann::json& j, button& p );


void to_json( nlohmann::json& j, const sync_laser& p );
void from_json( const nlohmann::json& j, sync_laser& p );
std::ostream& operator<<( std::ostream& os, sync_laser const& value );
} // namespace configuration
} // namespace raspigcd