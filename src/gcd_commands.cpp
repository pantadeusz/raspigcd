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


#include "gcd_commands.hpp"

#include "machine.hpp"

#include <functional>
#include <utility>      // std::pair
namespace tp {
namespace gcd {

void GcdCommandsInterpreter::setMachine(std::shared_ptr <  tp::motor::i_Machine > _machine) {
	machine_ = _machine;
}

void GcdCommandsInterpreter::setLine( int line ) {
	line_ = line;
}


void GcdCommandsInterpreter::breakExecution( ) {
	machine_.get()->breakExecution();
}

Position GcdCommandsInterpreter::getPosition(bool sync) {
	return 	machine_.get()->getPosition(sync);
}


// executes gcode command and after the command has been commited it executes callback function
void GcdCommandsInterpreter::execCommand( const std::string &gcd, std::function < void ( int, const std::string&, const std::string& ) > callback_ ) {
	std::string result;
	line_++;
	int i = 0;

	auto skipWhitespaces = [&](){
		while ( ( i < ( int )gcd.length() ) && ( ( gcd[i] == ' ' ) || ( gcd[i] == '\n' ) || ( gcd[i] == '\r' ) ) ) i++;
	};

	if ( ( gcd.length() == 0 ) || ( gcd[0] == ';' ) ) {
		result = "ok comment";
	} else {
		std::vector < std::pair < char, std::string > > commandSequence;
		char cmndCode = ' ';

		while ( i < ( int )gcd.length() ) {

			skipWhitespaces();
			if ( i < ( int )gcd.length() ) {
				if ( ( gcd[i] >= 'A' ) && ( gcd[i] <= 'Z' ) ) {
					cmndCode = gcd[i];
					i++;
				} else {
					result = "err bad gcode command";
					i = gcd.length();
					break;
				}
			}
			skipWhitespaces();

			std::string codeNumS;
			while ( ( i < ( int )gcd.length() ) && ( ( ( gcd[i] >= '0' ) && ( gcd[i] <= '9' ) ) || ( gcd[i] == '-' ) || ( gcd[i] == '.' ) ) ) {
				codeNumS = codeNumS + gcd[i];
				i++;
			}

			//std::pair < std::string, std::string > commandPartPair = std::make_pair(cmndCode,codeNumS);
			//std::cout << "K:" << cmndCode << " : " << codeNumS << std::endl;

			commandSequence.push_back( std::make_pair( cmndCode,codeNumS ) );
		}

		std::vector < std::map <char, double> > subCommandsV;
		std::vector < char > subCommandsN;
		int subCommandsIdx = -1;
		for ( auto cmnd : commandSequence ) {
			if ( subCommandsIdx >= 0 ) subCommandsV[subCommandsIdx][cmnd.first] = std::stod( cmnd.second );
			if ( executors.count( cmnd.first ) ) {
				std::map <char, double> m = {{cmnd.first, std::stod( cmnd.second )}};
				subCommandsIdx++;
				subCommandsN.push_back( cmnd.first );
				subCommandsV.push_back( m );
			}
		}

		for ( int i = 0; i <= subCommandsIdx; i++ ) {
			result = result + ((i>0)?"; ":"") + executors[subCommandsN[i]]( subCommandsV[i] ) ;
		}

	}


	callback_( line_-1, gcd, result );
}

void GcdCommandsInterpreter::finish() {
	machine_.get()->waitFinish();
}


GcdCommandsInterpreter::GcdCommandsInterpreter() {

	G0speed = 20;
	G1speed = 5;
	frMultiplier = 1; // 60 for mm/min, 1 for mm/s
	

	executors['G'] = [this]( std::map <char, double> &m ) -> std::string {
		Position p;
		if (( ( int )m['G'] == 0 ) || ( ( int )m['G'] == 1 )) { // work move
			int coordsToMove = 0;
			coordsToMove += m.count( 'X' )*1;
			coordsToMove += m.count( 'Y' )*2;
			coordsToMove += m.count( 'Z' )*4;
			Position pos( m.count( 'X' )?m['X']:0,m.count( 'Y' )?m['Y']:0,m.count( 'Z' )?m['Z']:0 );
			if ( ( int )m['G'] == 0 ) {
				if ( m.count( 'F' ) == 0 ) {
					m['F'] = G0speed*frMultiplier;
				} else {
					G0speed = m['F']/frMultiplier;
				}
			}
			if ( ( int )m['G'] == 1 ) {
				if ( m.count( 'F' ) == 0 ) {
					m['F'] = G1speed*frMultiplier;
				} else {
					G1speed = m['F']/frMultiplier;
				}
			}
			machine_.get()->gotoXYZ( pos, m['F']/frMultiplier, coordsToMove, G0speedV0, G0speedV0ddt ); // fast movement
			return "ok executed movement";
		} else if ( ( int )m['G'] == 4 ) {
			int t = 0;
			if ( m.count( 'P' ) == 1 ) {
				t = m['P'];
			} else if (m.count( 'X' ) == 1) {
				t = 1000*m['X'];
			}
			machine_.get()->pause(t) ;
			return "ok dwell executed";
		} else if (( int )m['G'] == 92) {
			// G92 -- reset coordinates
			if (m.count('X') == 0) m['X'] = 0;
			if (m.count('Y') == 0) m['Y'] = 0;
			if (m.count('Z') == 0) m['Z'] = 0;
			
			machine_.get()->setPosition( Position (m['X'],m['Y'],m['Z']) );
			return "ok reset position to X:"+std::to_string(m['X'])+" Y:"+std::to_string(m['Y'])+" Z:"+std::to_string(m['Z'])+"";
		
			
		} else if (( int )m['G'] == 28) {
			finish();
			// G28 -- move to origin
			if (!machine_.get()->isSteppersEnabled()) {
				return "!! steppers not enabled, can't move";	
			}
			if ((m.count('X') == 0) && (m.count('Y') == 0) && (m.count('Z') == 0)) {
				m['X'] = 0;
				m['Y'] = 0;
				m['Z'] = 0;
			}
			int axis_list[] = {2,0,1}; // z -> x -> y
			double axis_positive_list[] = {1.0,-1.0,1.0}; // z -> x -> y
			
			for (int t = 0; t < 3; t++) {
				int axis = axis_list[t];
				p = machine_.get()->getPosition(true);
				if (m.count('X'+axis)) {
					auto pNext = p;
					double n = 0;
					int i = 0;
					while ((machine_.get()->getEndstops()[axis] == 0) && (n < 400)) {
						n+= 0.5;
						pNext[axis] = p[axis]+n*axis_positive_list[t];
						machine_.get()->gotoXYZ( pNext, 20, 1+2+4, 20, 1 );
						finish();
					}
					while ((machine_.get()->getEndstops()[axis] == 0) && (i < 100)) {
						n-= 0.01;
						pNext[axis] = p[axis]+n*axis_positive_list[t];
						machine_.get()->gotoXYZ( pNext, 3, 1+2+4, 10, 1 );
						finish();
						i++;
					}
				}
			}

			p = machine_.get()->getPosition(true);
			return "ok origin X:" + std::to_string(p[0]) + " Y:" + std::to_string(p[1]) + " Z:" + std::to_string(p[2]) + "";
		
			// G92 -- reset coordinates
		} else return "!! unsupported G code number";
	};

	executors['M'] = [this]( std::map <char, double> &m ) -> std::string {
		Position p;
		std::array<unsigned char,4> endstops;
		int t = 0;
		switch( ( int )m['M'] ) {
		case 3:
			machine_.get()->spindleEnabled( true );
			t = 7000;
			if ( m.count( 'P' ) == 1 ) {
				t = m['P'];
			} else if (m.count( 'X' ) == 1) {
				t = 1000*m['X'];
			}
			if (t > 0) machine_.get()->pause(t) ;

			return "ok spindle on CW";
		case 5:
			machine_.get()->spindleEnabled( false );
			t = 0;
			if ( m.count( 'P' ) == 1 ) {
				t = m['P'];
			} else if (m.count( 'X' ) == 1) {
				t = 1000*m['X'];
			}
			if (t > 0) machine_.get()->pause(t) ;
			return "ok spindle off";
		case 17:
			machine_.get()->steppersEnable( true );
			return "ok steppers on";
		case 18:
			machine_.get()->steppersEnable( false );
			return "ok steppers off";
		case 114:
			p = machine_.get()->getPosition(true);
			return "ok position X:" + std::to_string(p[0]) + " Y:" + std::to_string(p[1]) + " Z:" + std::to_string(p[2]) + "";
		case 119:
			endstops = machine_.get()->getEndstops();
			return "ok endstops: X:" + std::to_string(endstops[0]) + " Y:" + std::to_string(endstops[1]) + " Z:" + std::to_string(endstops[2]) + " T:" + std::to_string(endstops[3]) + "";
		case 577:
			int btnn = machine_.get()->waitForEndstopTrigger(  );
			return "ok button pressed " + std::to_string(btnn);
		}
		return "!! unrecognized command";
	};
}


}
}







