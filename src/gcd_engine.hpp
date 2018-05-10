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
#include "m_motor_interface.hpp"
#include "m_motor_rpi.hpp"
#include <istream>
#include <vector>
#include <string>

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
		double g0speed;//: 160,
		double g1speed;//: 5,
		double g0speedV0;//:15,
		double g0speedV0ddt;//:1
	} GcdCommandsInterpreter;
	struct  {
		double tickTime;//: 100
	} MotorMoves;

	tp::motor::SpindlePiConfig spindle;
	std::vector < tp::motor::StepperPiConfig > stepper;
	tp::motor::ButtonsPiConfig buttons;
};

GcodeEngineConfig getDefaults_GcodeEngineConfig();

void to_json( nlohmann::json& j, const GcodeEngineConfig& p );
void from_json( const nlohmann::json& j, GcodeEngineConfig& p );
std::ostream& operator << ( std::ostream& os, GcodeEngineConfig const& value );
std::istream& operator >> ( std::istream& is, GcodeEngineConfig & value );
bool operator == ( const GcodeEngineConfig & l, const GcodeEngineConfig & r );


class GcodeEngine {
protected:
	std::shared_ptr<GcdCommandsInterpreter> p_gcd;
	std::mutex m_executingGcodeProgram; // shared mutex for gcode engine
	std::condition_variable cv_executingGcodeProgram; // cond variable that allows for notification that gcode is already executing
	bool execGcodeProgram_running;

	void setupGcdCommandsInterpreter( std::shared_ptr < tp::motor::i_MotorMoves > p_motor, const GcodeEngineConfig &configuration );
public:
	// method for breaking currently executed program.
	void breakExecution();
	void setLine( int line );

	// executes gcode command and after the command has been commited it executes callback function
	void execCommand( const std::string &command, std::function < void ( int, const std::string&, const std::string& ) > callback_ = []( int, const std::string &, const std::string & ) {} );

	void finish();
	bool isGcodeProgramRunning();
	Position getPosition();

	int execGcodeProgram( const std::string &gcodeProgram, std::function < void ( int, const std::string&, const std::string& ) > callback_ = []( int, const std::string &, const std::string & ) {}, std::function < void ( int ) > callback_end_ = []( int ) {} );
	int execGcodeProgram( std::istream &gcodeProgramSource, std::function < void ( int, const std::string&, const std::string& ) > callback_ = []( int, const std::string &, const std::string & ) {}, std::function < void ( int ) > callback_end_ = []( int ) {} );

	GcodeEngine ( std::shared_ptr < tp::motor::i_MotorMoves > p_motor, const GcodeEngineConfig &configuration );
	GcodeEngine ( const GcodeEngineConfig &configuration );
};


}
}

#endif



