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

/**
 * @brief This is the example of how to use raspigcd library
 * 
 * The main application for executing g-code on Raspberry Pi 2 and newer. It
 * shows how to use raspigcd library, especially GcodeEngine object to run gcode.
 * 
 * @file a_main.cpp
 * @author your name
 * @date 2018-06-23
 */



#include "raspigcd.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <streambuf>
#include <thread>

using std::chrono::seconds; // nanoseconds, system_clock, seconds
using std::this_thread::sleep_for; // sleep_for, sleep_until


int main(int argc, char** argv)
{
    /// Loading the configuration. First load some defaults from local file
    static std::string config = R"({"config": {}})";
    {
        std::ifstream configFile("defaults.json");
        if (configFile.is_open()) {
            config = std::string((std::istreambuf_iterator<char>(configFile)), std::istreambuf_iterator<char>());
        }
    }
    /// Loading the configuration from config.json - this is custom config overwriting all the defaults from defaults.json
    {
        std::ifstream configFile("config.json");
        if (configFile.is_open()) {
            config = std::string((std::istreambuf_iterator<char>(configFile)), std::istreambuf_iterator<char>());
        }
    }

    /// prepare configuration object from configuration string
    tp::gcd::GcodeEngineConfig gcdwconfig = nlohmann::json::parse(config);

    if (argc > 1) {
        // parse options, especially find some usefull json paths to overwrite config options
        for (int i = 2; i < argc; i++) {
            std::string arg(argv[i]);
            std::string jpath = arg.substr(0, arg.find('='));
            std::string jval = arg.substr(arg.find('=') + 1);
            nlohmann::json cfg = gcdwconfig;
            nlohmann::json j_patch = R"([])"_json;
            j_patch[0]["op"] = "replace";
            j_patch[0]["path"] = jpath;
            try {
                j_patch[0]["value"] = std::stof(jval);
            } catch (...) {
                j_patch[0]["value"] = jval;
            }
            // overwrite selected argument
            gcdwconfig = cfg.patch(j_patch);
        }
        /// print config - for reference and for someone wanting to get default or modified config file
        std::cout << gcdwconfig << std::endl;

        /// *************************************************************** ///
        /// Here is the magic - create gcd engine for g-code interpretation ///
        /// *************************************************************** ///
        tp::gcd::GcodeEngine gcdw(gcdwconfig);
        std::ifstream g(argv[1]); // file to read - gcode
        if (g.is_open()) {
            /// execute selected file. Note that the lambda provided is used
            /// only to print debug information. You can safely remove it.
            /// it is also possible to handle break commands during execution, 
            /// or measuring some dimensions of the machine and so on in callback.
            /// The code execution can be run in thread, and can be interrupted 
            /// at any time by executing gcdw.breakExecution()
            gcdw.execGcodeProgram(g, [](int l, const std::string& c, const std::string& r) { 
                std::cout << "(" << l << ") " << c << " -> " << r << std::endl; 
                }, [](int code) { 
                    std::cout << "gcode finished " << code << std::endl; 
                });
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
