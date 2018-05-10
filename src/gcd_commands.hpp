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


#ifndef ___GCD_COMMANDS_HPP___
#define ___GCD_COMMANDS_HPP___

#include "machine.hpp"

#include <functional>
#include <string>
#include <utility>
#include <map>

namespace tp {
namespace gcd {
using tp::coord::Position;

class GcdCommandsInterpreter {
protected:
	std::shared_ptr < tp::motor::i_Machine > machine_;
	int line_;

	double G0speed;
	double G1speed;
	double frMultiplier; // 60 for mm/min, 1 for mm/s, default is 1
	double G0speedV0; // the maximal speed that does not require acceleration
	double G0speedV0ddt; // how fast is acceleration (in microseconds per step)
	std::map<char, std::function < std::string ( std::map <char, double> & ) > > executors;


public:
	const double &workSpeed() const {
		return G1speed;
	}
	const double &fastSpeed() const {
		return G0speed;
	}
	const double &workSpeed( const double g1 ) {
		G1speed = g1;
		return G1speed;
	}
	const double &fastSpeed( const double g0 ) {
		G0speed = g0;
		return G0speed;
	}

	void g0speedAcc(double v0, double v0ddt) {
		G0speedV0 = v0; // the maximal speed that does not require acceleration
		G0speedV0ddt = v0ddt; // how fast is acceleration (in microseconds per step)
	}

	Position getPosition(bool sync = false);
	

	void setMachine( std::shared_ptr <  tp::motor::i_Machine > _machine );

	// gcode line, it increments every command is sent
	void setLine( int line );

	// executes gcode command and after the command has been commited it executes callback function
	void execCommand( const std::string &command, std::function < void ( int, const std::string&, const std::string& ) > callback_ = []( int, const std::string &, const std::string & ) {} );

	void finish();

	/**
	 This function allows for termination of commands - it will make the interpreter ignore every command.
	 */
	void breakExecution();

	GcdCommandsInterpreter();
};

}
}

#endif


