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

#ifndef __TP_MOTOR_DTO_STEPPERPICONFIG__
#define __TP_MOTOR_DTO_STEPPERPICONFIG__

#include <nlohmann/json.hpp>

namespace tp {
namespace motor {
    struct StepperPiConfig {
        unsigned int step;
        unsigned int dir;
        unsigned int en;
    };
    void to_json(nlohmann::json& j, const StepperPiConfig& p);
    void from_json(const nlohmann::json& j, StepperPiConfig& p);
    inline bool operator==(const StepperPiConfig& l, const StepperPiConfig& r)
    {
        return (l.step == r.step) && (l.dir == r.dir) && (l.en == r.en);
    };

    inline std::ostream& operator<<(std::ostream& os, StepperPiConfig const& value)
    {
        os << value.step << ", " << value.dir << ", " << value.en;
        return os;
    }
} // namespace motor
} // namespace tp

#endif
