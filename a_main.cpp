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

#include "coordsystem.hpp"
#include "gcd_engine.hpp"
#include "m_hwmoves.hpp"
#include "m_motor_interface.hpp"
#include "m_motor_rpi.hpp"
#include "machine.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <streambuf>
#include <thread>

using std::chrono::seconds; // nanoseconds, system_clock, seconds
using std::this_thread::sleep_for; // sleep_for, sleep_until

using tp::motor::i_Stepper;
using tp::motor::p_Stepper;
using tp::motor::StepperPi_factory;
using tp::motor::StepperPiConfig;

using tp::motor::i_Spindle;
using tp::motor::p_Spindle;
using tp::motor::SpindlePi_factory;
using tp::motor::SpindlePiConfig;

using tp::coord::CoordTranslate_corexy_factory;
using tp::coord::Position;
using tp::motor::i_MotorMoves;
using tp::motor::Machine;
using tp::motor::MotorCommand;
using tp::motor::MotorMoves_factory;
using tp::motor::Steps;

int main(int argc, char** argv)
{
    static std::string config = R "({" config ": {}})";
    {
        std::ifstream configFile("defaults.json");
        if (configFile.is_open()) {
            config = std::string((std::istreambuf_iterator<char>(configFile)), std::istreambuf_iterator<char>());
        }
    }
    {
        std::ifstream configFile("config.json");
        if (configFile.is_open()) {
            config = std::string((std::istreambuf_iterator<char>(configFile)), std::istreambuf_iterator<char>());
        }
    }

    tp::gcd::GcodeEngineConfig gcdwconfig = nlohmann::json::parse(config);

    if (argc > 1) {

        for (int i = 2; i < argc; i++) {
            std::string arg(argv[i]);
            std::string jpath = arg.substr(0, arg.find('='));
            std::string jval = arg.substr(arg.find('=') + 1);
            nlohmann::json cfg = gcdwconfig;
            nlohmann::json j_patch = R "([])" _json;
            j_patch[0]["op"] = "replace";
            j_patch[0]["path"] = jpath;
            try {
                j_patch[0]["value"] = std::stod(jval);
            } catch (...) {
                j_patch[0]["value"] = jval;
            }
            gcdwconfig = cfg.patch(j_patch);
        }
        std::cout << gcdwconfig << std::endl;

        tp::gcd::GcodeEngine gcdw(gcdwconfig);
        std::ifstream g(argv[1]);
        if (g.is_open()) {
            gcdw.execGcodeProgram(g, [](int l, const std::string& c, const std::string& r) { std::cout << "(" << l << ") " << c << " -> " << r << std::endl; }, [](int code) { std::cout << "gcode finished " << code << std::endl; });
            return 0;
        } else {
            return 1;
        }
    } else {
        std::cout << "Usage: " << argv[0] << " FILE [/config/path/to/option=value] [...]" << std::endl;
        std::cout << "Executes gcode file named FILE." << std::endl;
        std::cout << "You can provide any configuration option as [/config/path/to/option=value]" << std::endl;
        std::cout << "Exit status:" << std::endl;
        std::cout << " 0  if OK," << std::endl;
        std::cout << " 1  if file does not exist." << std::endl;
        std::cout << " 2  if there was no input file." << std::endl;
    }

    return 2;
}
