
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


#include "dto_GcodeEngineConfig.hpp"

namespace tp {
namespace gcd {

    GcodeEngineConfig getDefaults_GcodeEngineConfig()
    {

        GcodeEngineConfig ret;
        ret.GcodeEngine.simulationFileOutput = ""; // no file output
        ret.GcodeEngine.toolD = 2;
        ret.GcodeEngine.dpi = 75;

        ret.CoordTranslate = tp::coord::getDefaults_CoordTranslateConfig();

        ret.GcdCommandsInterpreter.g0speed = 160;
        ret.GcdCommandsInterpreter.g1speed = 5;
        ret.GcdCommandsInterpreter.g0speedV0 = 15;
        ret.GcdCommandsInterpreter.g0speedV0ddt = 10;
        ret.MotorMoves.tickTime = 100;
        ret.spindle.pin = 18;
        ret.spindle.servopwm = 1;
        ret.stepper.resize(4);
        ret.stepper[0].dir = 27;
        ret.stepper[0].en = 10;
        ret.stepper[0].step = 22;
        ret.stepper[1].dir = 4;
        ret.stepper[1].en = 10;
        ret.stepper[1].step = 17;
        ret.stepper[2].dir = 9;
        ret.stepper[2].en = 10;
        ret.stepper[2].step = 11;

        ret.stepper[3].dir = 0;
        ret.stepper[3].en = 10;
        ret.stepper[3].step = 5;

        //	ret.buttons.x             = 6;
        //	ret.buttons.y             = 13;
        //	ret.buttons.z             = 19;
        //	ret.buttons.t             = 26;
        ret.buttons.x = 21; // ok
        ret.buttons.y = 20;
        ret.buttons.z = 16;
        ret.buttons.t = 12;

        return ret;
    }

    void to_json(nlohmann::json& j, const GcodeEngineConfig& p)
    {
        nlohmann::json coordTranslateJson = p.CoordTranslate;
        nlohmann::json spindlePiConfigJson = p.spindle;
        j = nlohmann::json{
            { "config",
                { { "GcodeEngine", { { "simulationFileOutput", p.GcodeEngine.simulationFileOutput }, { "toolD", p.GcodeEngine.toolD }, { "dpi", p.GcodeEngine.dpi } } },
                    { "GcdCommandsInterpreter", {
                                                    { "g0speed", p.GcdCommandsInterpreter.g0speed }, //: 160,
                                                    { "g1speed", p.GcdCommandsInterpreter.g1speed }, //: 5,
                                                    { "g0speedV0", p.GcdCommandsInterpreter.g0speedV0 }, //:15,
                                                    { "g0speedV0ddt", p.GcdCommandsInterpreter.g0speedV0ddt } //:1
                                                } },
                    { "MotorMoves", {
                                        { "tickTime", p.MotorMoves.tickTime } //: 100
                                    } },
                    { "StepperPiConfig", { { "m0", p.stepper[0] }, { "m1", p.stepper[1] }, { "m2", p.stepper[2] }, { "m3", p.stepper[3] } } }, { "ButtonsPiConfig", p.buttons } } }
        };
        j["config"]["CoordTranslate"] = coordTranslateJson;
        j["config"]["SpindlePiConfig"] = spindlePiConfigJson;
    }

    void from_json(const nlohmann::json& j, GcodeEngineConfig& p)
    {
        p = getDefaults_GcodeEngineConfig();
        try {
            p.GcodeEngine.simulationFileOutput = j.at("config").at("GcodeEngine").at("simulationFileOutput").get<std::string>();
        } catch (...) {
        }
        try {
            p.GcodeEngine.toolD = j.at("config").at("GcodeEngine").at("toolD").get<double>();
        } catch (...) {
        }
        try {
            p.GcodeEngine.dpi = j.at("config").at("GcodeEngine").at("dpi").get<double>();
        } catch (...) {
        }
        try {
            p.GcdCommandsInterpreter.g0speed = j.at("config").at("GcdCommandsInterpreter").at("g0speed").get<double>();
        } catch (...) {
        }; // 160
        try {
            p.GcdCommandsInterpreter.g1speed = j.at("config").at("GcdCommandsInterpreter").at("g1speed").get<double>();
        } catch (...) {
        }; // 5
        try {
            p.GcdCommandsInterpreter.g0speedV0 = j.at("config").at("GcdCommandsInterpreter").at("g0speedV0").get<double>();
        } catch (...) {
        }; // 15
        try {
            p.GcdCommandsInterpreter.g0speedV0ddt = j.at("config").at("GcdCommandsInterpreter").at("g0speedV0ddt").get<double>();
        } catch (...) {
        }; // 1
        try {
            p.MotorMoves.tickTime = j.at("config").at("MotorMoves").at("tickTime").get<double>();
        } catch (...) {
        }; // 100

        try {
            p.CoordTranslate.motorConfiguration = j.at("config").at("CoordTranslate").at("motorConfiguration");
        } catch (...) {
        }; // corexy
        try {
            p.CoordTranslate.scale.x = j.at("config").at("CoordTranslate").at("scale").at("x");
        } catch (...) {
        }; // -1.0
        try {
            p.CoordTranslate.scale.y = j.at("config").at("CoordTranslate").at("scale").at("y");
        } catch (...) {
        }; // 1.0
        try {
            p.CoordTranslate.scale.z = j.at("config").at("CoordTranslate").at("scale").at("z");
        } catch (...) {
        }; // 1.0
        try {
            p.CoordTranslate.stepsPerMm.a = j.at("config").at("CoordTranslate").at("stepsPerMm").at("a");
        } catch (...) {
        }; // 82.0512820513
        try {
            p.CoordTranslate.stepsPerMm.b = j.at("config").at("CoordTranslate").at("stepsPerMm").at("b");
        } catch (...) {
        }; // 82.0512820513
        try {
            p.CoordTranslate.stepsPerMm.c = j.at("config").at("CoordTranslate").at("stepsPerMm").at("c");
        } catch (...) {
        }; // 200

        try {
            p.spindle.pin = j.at("config").at("SpindlePiConfig").at("pin").get<int>();
        } catch (...) {
        }; // 18
        try {
            p.spindle.servopwm = j.at("config").at("SpindlePiConfig").at("servopwm").get<int>();
        } catch (...) {
        }; // 1

        for (int i = 0; i < 3; i++) {
            try {
                p.stepper[i].step = j.at("config").at("StepperPiConfig").at(std::string("m") + std::to_string(i)).at("step");
            } catch (...) {
            }
            try {
                p.stepper[i].dir = j.at("config").at("StepperPiConfig").at(std::string("m") + std::to_string(i)).at("dir");
            } catch (...) {
            }
            try {
                p.stepper[i].en = j.at("config").at("StepperPiConfig").at(std::string("m") + std::to_string(i)).at("en");
            } catch (...) {
            }
        }

        try {
            p.buttons = j.at("config").at("ButtonsPiConfig");
        } catch (...) {
        }; // 18
    }

    std::ostream& operator<<(std::ostream& os, GcodeEngineConfig const& value)
    {
        nlohmann::json j;
        j = value;
        os << j.dump();
        return os;
    }

    std::istream& operator>>(std::istream& is, GcodeEngineConfig& value)
    {
        nlohmann::json j;
        is >> j;
        value = j;
        return is;
    }
    bool operator==(const GcodeEngineConfig& l, const GcodeEngineConfig& r)
    {
        if (l.CoordTranslate.motorConfiguration != r.CoordTranslate.motorConfiguration)
            return false;
        if (!(l.CoordTranslate == r.CoordTranslate))
            return false;
        if (l.GcdCommandsInterpreter.g0speed != r.GcdCommandsInterpreter.g0speed)
            return false;
        if (l.GcdCommandsInterpreter.g1speed != r.GcdCommandsInterpreter.g1speed)
            return false;
        if (l.GcdCommandsInterpreter.g0speedV0 != r.GcdCommandsInterpreter.g0speedV0)
            return false;
        if (l.GcdCommandsInterpreter.g0speedV0ddt != r.GcdCommandsInterpreter.g0speedV0ddt)
            return false;
        if (l.MotorMoves.tickTime != r.MotorMoves.tickTime)
            return false;
        if (l.spindle.pin != r.spindle.pin)
            return false;
        if (l.spindle.servopwm != r.spindle.servopwm)
            return false;
        if (!(l.stepper[0] == r.stepper[0]))
            return false;
        if (!(l.stepper[1] == r.stepper[1]))
            return false;
        if (!(l.stepper[2] == r.stepper[2]))
            return false;
        return true;
    }

} // namespace gcd
} // namespace tp