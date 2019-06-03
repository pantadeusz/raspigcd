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

#ifndef __TP_GCD_DTO_GCODE_ENGINE_CONFIG_HPP__
#define __TP_GCD_DTO_GCODE_ENGINE_CONFIG_HPP__

#include "s_GcdCommandsInterpreter.hpp"
#include "i_Stepper.hpp"
#include "i_Spindle.hpp"
#include "i_Buttons.hpp"
#include "driver_raspberry_pi.hpp"
#include <istream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace tp {
namespace gcd {

    /**
 * @brief Complete gcode engine configuration. Parts of it goes to different elements configured!
 *
 ************/

    struct GcodeEngineConfig {
        tp::coord::CoordTranslateConfig CoordTranslate;
        struct {
            std::string simulationFileOutput;
            double toolD;
            double dpi;
        } GcodeEngine;
        struct {
            double g0speed; //: 160,
            double g1speed; //: 5,
            double g0speedV0; //:15,
            double g0speedV0ddt; //:1
        } GcdCommandsInterpreter;
        struct {
            double tickTime; //: 100
        } MotorMoves;

        tp::motor::SpindlePiConfig spindle;
        std::vector<tp::motor::StepperPiConfig> stepper;
        tp::motor::ButtonsPiConfig buttons;
    };

    GcodeEngineConfig getDefaults_GcodeEngineConfig();

    void to_json(nlohmann::json& j, const GcodeEngineConfig& p);
    void from_json(const nlohmann::json& j, GcodeEngineConfig& p);
    std::ostream& operator<<(std::ostream& os, GcodeEngineConfig const& value);
    std::istream& operator>>(std::istream& is, GcodeEngineConfig& value);
    bool operator==(const GcodeEngineConfig& l, const GcodeEngineConfig& r);

} // namespace gcd
} // namespace tp

#endif
