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


#ifndef __MACHINE__HPP____
#define  __MACHINE__HPP____

#include "coordsystem.hpp"
#include "m_hwmoves.hpp"
#include <atomic>

namespace tp {
namespace motor {
using tp::coord::Position;


class i_Machine {
public:
	class BreakException {
	public:
		std::string why;
		BreakException( const std::string &w = "stopped" ) {
			why = w;
		}
	};
	virtual void breakExecution() = 0;

	// goes to selected position
	virtual void gotoXYZ( const Position &pos, double v, int selectedAxes = 0x0ff, double maxVelocityNoAccel = -1, double ddt = 100 ) = 0;

	// enables or disables spindle
	virtual void spindleEnabled( bool _enabled ) = 0;

	// set steppers on or off
	virtual void steppersEnable( bool _enabled ) = 0;

	// get steppers state
	virtual bool isSteppersEnabled(  ) = 0;

	// pause for given time
	virtual void pause( int ms_time ) = 0;

	// resets the position
	virtual void setPosition( const Position &pos ) = 0;
	// returns the current position
	virtual Position getPosition(bool sync = true)  = 0;

	virtual int waitForEndstopTrigger() = 0;

	virtual std::array<unsigned char,4> getEndstops() = 0;

	// waits for commands to finish
	virtual void waitFinish() = 0;
};


class Machine : public i_Machine {
protected:
	std::shared_ptr < tp::coord::i_CoordTranslate > coordTranslator_;
	std::shared_ptr < i_MotorMoves > motorMoves_;
	Position minimalStepMm; // this is the positive vector for minimal step in direction (1,1,1,1) in steps

	std::mutex m_csteps_;
	Steps currentSteps_; // current position in steps

	std::atomic_bool doBreak;

public:
	void breakExecution();

	// goes to selected position
	void gotoXYZ( const Position &pos, double v, int selectedAxes = 0x0ff, double maxVelocityNoAccel = -1, double ddt = 100 );

	// enables or disables spindle
	void spindleEnabled( bool _enabled );

	// set steppers on or off
	void steppersEnable( bool _enabled );

	bool isSteppersEnabled(  );

	// pause for given time
	void pause( int ms_time );

	// resets the position
	void setPosition( const Position &pos );
	// returns the current position
	Position getPosition(bool sync = true) ;

	// waits for commands to finish
	void waitFinish();
	// configuration of hardware layer
	void setMotorMoves( std::shared_ptr < i_MotorMoves > _motorMoves ) ;
	// gets the coordinate system translation -- this will make smart pointers out of it
	void setCoordinateSystem( std::shared_ptr < tp::coord::i_CoordTranslate > _coordTranslator );

	int waitForEndstopTrigger();

	std::array<unsigned char,4> getEndstops();


	Machine() {doBreak = false;}
};

}
}

#endif


