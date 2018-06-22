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

#ifndef __MACHINE_PI__HPP____
#define __MACHINE_PI__HPP____

#include "gcd_commands.hpp"
#include "i_Stepper.hpp"
#include "i_Spindle.hpp"
#include "i_Buttons.hpp"
#include "m_motor_rpi.hpp"

#include "dto_GcodeEngineConfig.hpp"

#include <istream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace tp {
namespace gcd {

    class GcodeEngine {
    protected:
        std::shared_ptr<GcdCommandsInterpreter> p_gcd;
        std::mutex m_executingGcodeProgram; // shared mutex for gcode engine
        std::condition_variable cv_executingGcodeProgram; // cond variable that allows for notification that gcode is already executing
        bool execGcodeProgram_running;

        void setupGcdCommandsInterpreter(std::shared_ptr<tp::motor::i_MotorMoves> p_motor, const GcodeEngineConfig& configuration);

    public:
        // method for breaking currently executed program.
        void breakExecution();
        void setLine(int line);

        // executes gcode command and after the command has been commited it executes callback function
        void execCommand(const std::string& command, std::function<void(int, const std::string&, const std::string&)> callback_ = [](int, const std::string&, const std::string&) {});

        void finish();
        bool isGcodeProgramRunning();
        Position getPosition();

        int execGcodeProgram(const std::string& gcodeProgram, std::function<void(int, const std::string&, const std::string&)> callback_ = [](int, const std::string&, const std::string&) {}, std::function<void(int)> callback_end_ = [](int) {});
        int execGcodeProgram(std::istream& gcodeProgramSource, std::function<void(int, const std::string&, const std::string&)> callback_ = [](int, const std::string&, const std::string&) {}, std::function<void(int)> callback_end_ = [](int) {});

        GcodeEngine(std::shared_ptr<tp::motor::i_MotorMoves> p_motor, const GcodeEngineConfig& configuration);
        GcodeEngine(const GcodeEngineConfig& configuration);
    };

} // namespace gcd
} // namespace tp

#endif
