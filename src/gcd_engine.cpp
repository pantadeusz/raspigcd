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


#include "gcd_engine.hpp"
#include "m_motor_rpi.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <list>
#include <regex>
#include <tpcommon/img_img8.hpp>

namespace tp {
namespace gcd {

using json = nlohmann::json;

using tp::motor::i_Stepper;
using tp::motor::p_Stepper;
using tp::motor::StepperPiConfig;
using tp::motor::StepperPi_factory;

using tp::motor::i_Spindle;
using tp::motor::p_Spindle;
using tp::motor::SpindlePiConfig;
using tp::motor::SpindlePi_factory;

using tp::motor::i_Buttons;
using tp::motor::p_Buttons;
using tp::motor::ButtonsPiConfig;
using tp::motor::ButtonsPi_factory;


using tp::motor::MotorMoves_factory;
using tp::motor::i_MotorMoves;
using tp::motor::i_Machine;
using tp::motor::MotorCommand;
using tp::motor::Steps;
using tp::motor::Machine;
using tp::coord::Position;
using tp::coord::CoordTranslate_corexy_factory;
using tp::coord::CoordTranslate_simple_factory;


GcodeEngineConfig getDefaults_GcodeEngineConfig() {

	GcodeEngineConfig ret;
	ret.GcodeEngine.simulationFileOutput   = ""; // no file output
	ret.GcodeEngine.toolD                  = 2;
	ret.GcodeEngine.dpi                    = 75;

	ret.CoordTranslate = tp::coord::getDefaults_CoordTranslateConfig();

	ret.GcdCommandsInterpreter.g0speed      = 160;
	ret.GcdCommandsInterpreter.g1speed      = 5;
	ret.GcdCommandsInterpreter.g0speedV0    = 15;
	ret.GcdCommandsInterpreter.g0speedV0ddt = 10;
	ret.MotorMoves.tickTime                 = 100;
	ret.spindle.pin                 = 18;
	ret.spindle.servopwm            = 1;
	ret.stepper.resize( 4 );
	ret.stepper[0].dir              = 27;
	ret.stepper[0].en               = 10;
	ret.stepper[0].step             = 22;
	ret.stepper[1].dir              = 4;
	ret.stepper[1].en               = 10;
	ret.stepper[1].step             = 17;
	ret.stepper[2].dir              = 9;
	ret.stepper[2].en               = 10;
	ret.stepper[2].step             = 11;

	ret.stepper[3].dir              = 0;
	ret.stepper[3].en               = 10;
	ret.stepper[3].step             = 5;

//	ret.buttons.x             = 6;
//	ret.buttons.y             = 13;
//	ret.buttons.z             = 19;
//	ret.buttons.t             = 26;
	ret.buttons.x             = 21; // ok
	ret.buttons.y             = 20;
	ret.buttons.z             = 16;
	ret.buttons.t             = 12;


	return ret;
}

void to_json( nlohmann::json& j, const GcodeEngineConfig& p ) {
	nlohmann::json coordTranslateJson = p.CoordTranslate;
	nlohmann::json spindlePiConfigJson = p.spindle;
	j = nlohmann::json {
		{
			"config",
			{
				{
					"GcodeEngine", {
						{"simulationFileOutput", p.GcodeEngine.simulationFileOutput},
						{"toolD", p.GcodeEngine.toolD},
						{"dpi", p.GcodeEngine.dpi}
					}
				},
				{
					"GcdCommandsInterpreter", {
						{"g0speed", p.GcdCommandsInterpreter.g0speed}, //: 160,
						{"g1speed", p.GcdCommandsInterpreter.g1speed}, //: 5,
						{"g0speedV0", p.GcdCommandsInterpreter.g0speedV0}, //:15,
						{"g0speedV0ddt", p.GcdCommandsInterpreter.g0speedV0ddt} //:1
					}
				},
				{
					"MotorMoves", {
						{"tickTime", p.MotorMoves.tickTime} //: 100
					}
				},
				{
					"StepperPiConfig", {
						{
							"m0", p.stepper[0]
						},
						{
							"m1", p.stepper[1]
						},
						{
							"m2", p.stepper[2]
						},
						{
							"m3", p.stepper[3]
						}
					}
				},
				{
					"ButtonsPiConfig", p.buttons
				}
			}
		}
	};
	j["config"]["CoordTranslate"] = coordTranslateJson;
	j["config"]["SpindlePiConfig"] = spindlePiConfigJson;
}

void from_json( const nlohmann::json& j, GcodeEngineConfig& p ) {
	p = getDefaults_GcodeEngineConfig();
	try {
		p.GcodeEngine.simulationFileOutput    = j.at( "config" ).at( "GcodeEngine" ).at( "simulationFileOutput" ).get<std::string>();
	} catch ( ... ) {}
	try {
		p.GcodeEngine.toolD                   = j.at( "config" ).at( "GcodeEngine" ).at( "toolD" ).get<double>();
	} catch ( ... ) {}
	try {
		p.GcodeEngine.dpi                     = j.at( "config" ).at( "GcodeEngine" ).at( "dpi" ).get<double>();
	} catch ( ... ) {}
	try {
		p.GcdCommandsInterpreter.g0speed      = j.at( "config" ).at( "GcdCommandsInterpreter" ).at( "g0speed" ).get<double>();
	} catch ( ... ) {}          ; // 160
	try {
		p.GcdCommandsInterpreter.g1speed      = j.at( "config" ).at( "GcdCommandsInterpreter" ).at( "g1speed" ).get<double>();
	} catch ( ... ) {}          ; // 5
	try {
		p.GcdCommandsInterpreter.g0speedV0    = j.at( "config" ).at( "GcdCommandsInterpreter" ).at( "g0speedV0" ).get<double>();
	} catch ( ... ) {}          ; // 15
	try {
		p.GcdCommandsInterpreter.g0speedV0ddt = j.at( "config" ).at( "GcdCommandsInterpreter" ).at( "g0speedV0ddt" ).get<double>();
	} catch ( ... ) {}          ; // 1
	try {
		p.MotorMoves.tickTime                 = j.at( "config" ).at( "MotorMoves" ).at( "tickTime" ).get<double>();
	} catch ( ... ) {}          ; // 100


	try {
		p.CoordTranslate.motorConfiguration   = j.at( "config" ).at( "CoordTranslate" ).at( "motorConfiguration" )    ;
	} catch ( ... ) {}       ; // corexy
	try {
		p.CoordTranslate.scale.x              = j.at( "config" ).at( "CoordTranslate" ).at( "scale" ).at( "x" )         ;
	} catch ( ... ) {}       ; // -1.0
	try {
		p.CoordTranslate.scale.y              = j.at( "config" ).at( "CoordTranslate" ).at( "scale" ).at( "y" )         ;
	} catch ( ... ) {}       ; // 1.0
	try {
		p.CoordTranslate.scale.z              = j.at( "config" ).at( "CoordTranslate" ).at( "scale" ).at( "z" )         ;
	} catch ( ... ) {}       ; // 1.0
	try {
		p.CoordTranslate.stepsPerMm.a         = j.at( "config" ).at( "CoordTranslate" ).at( "stepsPerMm" ).at( "a" )    ;
	} catch ( ... ) {}       ; // 82.0512820513
	try {
		p.CoordTranslate.stepsPerMm.b         = j.at( "config" ).at( "CoordTranslate" ).at( "stepsPerMm" ).at( "b" )    ;
	} catch ( ... ) {}       ; // 82.0512820513
	try {
		p.CoordTranslate.stepsPerMm.c         = j.at( "config" ).at( "CoordTranslate" ).at( "stepsPerMm" ).at( "c" )    ;
	} catch ( ... ) {}       ; // 200


	try {
		p.spindle.pin                         = j.at( "config" ).at( "SpindlePiConfig" ).at( "pin" ).get<int>();
	} catch ( ... ) {}                       ; // 18
	try {
		p.spindle.servopwm                    = j.at( "config" ).at( "SpindlePiConfig" ).at( "servopwm" ).get<int>();
	} catch ( ... ) {}                       ; // 1

	for ( int i = 0; i < 3; i++ ) {
		try {
			p.stepper[i].step                          = j.at( "config" ).at( "StepperPiConfig" ).at( std::string( "m" ) + std::to_string( i ) ).at( "step" );
		} catch ( ... ) {}
		try {
			p.stepper[i].dir                           = j.at( "config" ).at( "StepperPiConfig" ).at( std::string( "m" ) + std::to_string( i ) ).at( "dir" );
		} catch ( ... ) {}
		try {
			p.stepper[i].en                            = j.at( "config" ).at( "StepperPiConfig" ).at( std::string( "m" ) + std::to_string( i ) ).at( "en" );
		} catch ( ... ) {}
	}


	try {
		p.buttons                         = j.at( "config" ).at( "ButtonsPiConfig" );
	} catch ( ... ) {}                       ; // 18


}


std::ostream& operator << ( std::ostream& os, GcodeEngineConfig const& value ) {
	nlohmann::json j;
	j = value;
	os << j.dump();
	return os;
}

std::istream& operator >> ( std::istream& is, GcodeEngineConfig & value ) {
	nlohmann::json j;
	is >> j;
	value = j;
	return is;
}
bool operator == ( const GcodeEngineConfig & l, const GcodeEngineConfig & r ) {
	if ( l.CoordTranslate.motorConfiguration   != r.CoordTranslate.motorConfiguration   ) return false;
	if ( !( l.CoordTranslate                   == r.CoordTranslate                     ) ) return false;
	if ( l.GcdCommandsInterpreter.g0speed      != r.GcdCommandsInterpreter.g0speed      ) return false;
	if ( l.GcdCommandsInterpreter.g1speed      != r.GcdCommandsInterpreter.g1speed      ) return false;
	if ( l.GcdCommandsInterpreter.g0speedV0    != r.GcdCommandsInterpreter.g0speedV0    ) return false;
	if ( l.GcdCommandsInterpreter.g0speedV0ddt != r.GcdCommandsInterpreter.g0speedV0ddt ) return false;
	if ( l.MotorMoves.tickTime                 != r.MotorMoves.tickTime                 ) return false;
	if ( l.spindle.pin                         != r.spindle.pin                         ) return false;
	if ( l.spindle.servopwm                    != r.spindle.servopwm                    ) return false;
	if ( !( l.stepper[0]                       == r.stepper[0]                         ) ) return false;
	if ( !( l.stepper[1]                       == r.stepper[1]                         ) ) return false;
	if ( !( l.stepper[2]                       == r.stepper[2]                         ) ) return false;
	return true;
}



class MachineImgGen: public i_Machine {
protected:
	Position currentPosition_; // current position in steps
	std::atomic_bool doBreak;
	std::string outputImage_;
	double dpi_;
	double toolD_;
	bool steppers_en_;
	std::list < Position > movesList;
public:
	void breakExecution() {
		doBreak = true;
	};

	void gotoXYZ( const Position &pos, double, int selectedAxes, double maxVelocityNoAccel, double ddt ) {
		auto newPos = currentPosition_;
		int p = 1;
		for ( int i = 0; i < 3; i++, p *= 2 ) {

			if ( selectedAxes & p ) {
				newPos[i] = pos[i];
			}
		}
		movesList.push_back( newPos );
		currentPosition_ = newPos;
	}

	void spindleEnabled( bool  ) {};

	// set steppers on or off
	void steppersEnable( bool e ) {steppers_en_ = e;};

  	bool isSteppersEnabled( ) {return steppers_en_;};

	// pause for given time
	void pause( int ) {};

	// resets the position
	void setPosition( const Position &pos ) {
		currentPosition_ = pos;
	};
	// returns the current position
	Position getPosition( bool ) {
		return currentPosition_;
	};

	// waits for commands to finish and then save the image
	void waitFinish() {

		Position minimums( 0, 0, 0 );
		Position maximums( 0, 0, 0 );
		double scaleXY = ( dpi_ * 0.0393701 );
		double toolR = toolD_ * 0.5 * scaleXY;
		if ( movesList.size() > 0 ) {
			auto p0 = minimums = maximums = movesList.front();//movesList[0];

			for ( auto p : movesList ) {
				for ( int i = 0; i < 3; i++ ) {
					if ( minimums[i] > p[i] ) minimums[i] = p[i];
					if ( maximums[i] < p[i] ) maximums[i] = p[i];
				}
			}
			double scaleZ = ( minimums[2] < 0 ) ? ( 1.0 / ( minimums[2] ) ) : -1;
			tp::img::Img8 newImg(  ( maximums[0] - minimums[0] )*scaleXY + toolR * 4, ( maximums[1] - minimums[1] )*scaleXY + toolR * 4  );
			newImg.setTo( 255 );

			for ( auto p : movesList ) {
				if ( ( p0[2] < 0 ) && ( p[2] < 0 ) ) {
					int x1, y1, x2, y2, r, c;
					x1 = ( p0[0] - minimums[0] + toolD_ ) * scaleXY,
					y1 = ( -p0[1] + maximums[1] + toolD_ ) * scaleXY;
					x2 = ( p[0] - minimums[0] + toolD_ ) * scaleXY;
					y2 = ( -p[1] + maximums[1] + toolD_ ) * scaleXY;
					r = ( toolR > 0.5 ) ? toolR : 0.5;
					c = 255.0 * ( 1.0 - p[2] * scaleZ );
					newImg.drawCircleLineToMin( x1, y1, x2, y2, r * 2.0, c );
				}
				p0 = p;
			}
			newImg.save( outputImage_ );
		}

		movesList.clear();
	};

	int waitForEndstopTrigger() {
		return 0;
	};
	std::array<unsigned char,4> getEndstops() {
		return {0,0,0,0};
	}

	MachineImgGen( const std::string &outputImage, double dpi = 75, double toolD = 2 ) {
		std::cout << "MacineImgGen " << outputImage << ", " << dpi << ", " << toolD << std::endl;
		doBreak = false;
		outputImage_ = outputImage;
		dpi_ = dpi;
		toolD_ = toolD;
	}

};






void GcodeEngine::setLine( int line ) {
	p_gcd.get()->setLine( line );
}
void GcodeEngine::execCommand( const std::string &command, std::function < void ( int, const std::string&, const std::string& ) > callback_ ) {
	p_gcd.get()->execCommand( command, callback_ );
}

void GcodeEngine::finish() {
	p_gcd.get()->finish();
}

void GcodeEngine::breakExecution() {
	p_gcd.get()->breakExecution();
}

bool GcodeEngine::isGcodeProgramRunning() {
	return execGcodeProgram_running;
}

Position GcodeEngine::getPosition() {
	return p_gcd.get()->getPosition( false );
}


int GcodeEngine::execGcodeProgram( std::istream &gcodeProgramSource, std::function < void ( int, const std::string&, const std::string& ) > callback_, std::function < void ( int ) > callback_end_ ) {
	//std::cout << "execGcodeProgram from stream" << std::endl;
	if ( execGcodeProgram_running ) {
		callback_end_( -1 );
		return -1;
	}
	try {
		std::string line;
		setLine( 0 );
		char c;
		while ( gcodeProgramSource.get( c ) ) {
			if ( ( c != '\n' ) && ( c != '\r' ) ) {
				line = line + c;
			} else {
				execCommand( line, callback_ );
				line = "";
			}
		}
		p_gcd.get()->finish();
		callback_end_( 0 );
		{
			std::lock_guard<std::mutex> lock( m_executingGcodeProgram );
			execGcodeProgram_running = false;
		}
		return 0;
	} catch ( Machine::BreakException e ) {
		//std::cout << "broken.." << std::endl;
		p_gcd.get()->finish();
		//std::cout << "broken..OK" << std::endl;
		callback_end_( -2 );
		{
			std::lock_guard<std::mutex> lock( m_executingGcodeProgram );
			execGcodeProgram_running = false;
		}
		return -2;
	} catch ( ... ) {
		callback_end_( -127 );
		{
			std::lock_guard<std::mutex> lock( m_executingGcodeProgram );
			execGcodeProgram_running = false;
		}
		return -127;
	}
}


int GcodeEngine::execGcodeProgram( const std::string &gcodeProgram, std::function < void ( int, const std::string&, const std::string& ) > callback_, std::function < void ( int ) > callback_end_ ) {
	std::stringstream str( gcodeProgram );
	return execGcodeProgram( str, callback_, callback_end_ );
}

void GcodeEngine::setupGcdCommandsInterpreter( std::shared_ptr < i_MotorMoves > p_motor, const GcodeEngineConfig &configuration ) {
	tp::gcd::GcdCommandsInterpreter *ret = new tp::gcd::GcdCommandsInterpreter();

	auto pp_machine = new Machine();
	std::shared_ptr < i_Machine > p_machine( pp_machine );
	pp_machine->setCoordinateSystem( CoordTranslate_factory( configuration.CoordTranslate )  );
	pp_machine->setMotorMoves( p_motor ) ;
	ret->setMachine( p_machine );

	ret->fastSpeed( configuration.GcdCommandsInterpreter.g0speed );
	ret->workSpeed( configuration.GcdCommandsInterpreter.g1speed );
	ret->g0speedAcc( configuration.GcdCommandsInterpreter.g0speedV0, configuration.GcdCommandsInterpreter.g0speedV0ddt );

	p_gcd = std::shared_ptr<GcdCommandsInterpreter>( ret );
}

GcodeEngine::GcodeEngine ( const GcodeEngineConfig &configuration ): execGcodeProgram_running( false ) {

	if ( configuration.GcodeEngine.simulationFileOutput.length() > 0 ) {
		//std::cout << "FAKE GCODE ENGINE!!  Saving to file: " << ( configuration.GcodeEngine.simulationFileOutput ) << std::endl;
		tp::gcd::GcdCommandsInterpreter *ret = new tp::gcd::GcdCommandsInterpreter();

		auto pp_machine = new MachineImgGen( configuration.GcodeEngine.simulationFileOutput, configuration.GcodeEngine.dpi, configuration.GcodeEngine.toolD );
		std::shared_ptr < i_Machine > p_machine( pp_machine );
		ret->setMachine( p_machine );

		ret->fastSpeed( configuration.GcdCommandsInterpreter.g0speed );
		ret->workSpeed( configuration.GcdCommandsInterpreter.g1speed );
		p_gcd = std::shared_ptr<GcdCommandsInterpreter>( ret );

		std::cout << "Created GCD engine that will save to a file " << configuration.GcodeEngine.simulationFileOutput << std::endl;
	} else {
		std::shared_ptr < i_MotorMoves > p_motor = MotorMoves_factory( 
			StepperPi_factory( configuration.stepper ), 
			SpindlePi_factory( configuration.spindle ), 
			ButtonsPi_factory( configuration.buttons ), 
			configuration.MotorMoves.tickTime );
		setupGcdCommandsInterpreter( p_motor, configuration );
	}
}


GcodeEngine::GcodeEngine ( std::shared_ptr < i_MotorMoves > p_motor, const GcodeEngineConfig &configuration ): execGcodeProgram_running( false ) {
	setupGcdCommandsInterpreter( p_motor, configuration );
}


}
}











