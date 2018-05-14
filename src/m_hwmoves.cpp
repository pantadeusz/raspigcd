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


#include "m_hwmoves.hpp"

#include "m_motor_interface.hpp"

#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <cstring>
#include <atomic>

#include <chrono>


using std::this_thread::sleep_for;
using std::this_thread::sleep_until;

using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::steady_clock;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

namespace tp {
namespace motor {

class MotorMoves : public i_MotorMoves {
protected:
	std::atomic_bool paused_;
	int _steps[4];
	// this objec does not own motors, it just takes pointers
	i_Stepper *_motors_p;
	i_Spindle *_spindle_p;
	i_Buttons *_buttons_p;

	// if smart pointers are passed, then we use them and duplicate to i_Stepper and i_Spindle
	p_Stepper _motors_sp;
	p_Spindle _spindle_sp;
	p_Buttons _buttons_sp;


	SafeQueue < MotorCommand > _commands;
	std::mutex m_steps_;

	std::thread worker_thread_;

	int _minStepTime;

	static void worker( MotorMoves *pThis );

public:

	inline i_Stepper *getMotors() {
		if ( _motors_sp.get() != NULL ) return _motors_sp.get();
		else return _motors_p;
	};

	inline i_Spindle *getSpindle() {
		if ( _spindle_sp.get() != NULL ) return _spindle_sp.get();
		else return _spindle_p;
	};

	void push( const MotorCommand & command );
	void wait_finished();
	Steps steps_from_origin();
	void steps_from_origin( Steps s );
	void reset_origin();
	int get_min_step_time_us();
	void clear_command_queue();
	void set_paused( bool paused );
	bool is_paused();
	bool is_steppers_enabled();
	std::array<unsigned char,4>  buttons_state();

	MotorMoves( i_Stepper* motors_, i_Spindle *spindle_, i_Buttons *buttons_,int minStepTime_ );
	MotorMoves( p_Stepper motors_p, p_Spindle spindle_p, p_Buttons buttons_,int minStepTime_ );

	virtual ~MotorMoves();
};

void MotorMoves::set_paused( bool paused ) {
	paused_ = paused;
}
bool MotorMoves::is_paused() {
	return paused_;
}

bool MotorMoves::is_steppers_enabled() {
	return _motors_p->is_enabled();
}

void MotorMoves::push( const MotorCommand & command_ ) {
	//std::this_thread::sleep_until ( steady_clock::now()+std::chrono::microseconds( 40 ) );
	_commands.push ( command_ );
}
void MotorMoves::clear_command_queue() {
	_commands.removeAll();
}

void MotorMoves::wait_finished() {
	MotorCommand command;
	command.delayBefore = 10;
	command.commands = MotorCommand::Command::nop;
	_commands.push ( command );
	_commands.waitEmpty();
//	std::this_thread::sleep_for ( std::chrono::microseconds( 10000000 ) );
	//std::unique_lock<std::mutex> lock( m_steps_ );
}
Steps MotorMoves::steps_from_origin() {
	std::unique_lock<std::mutex> l( m_steps_ );
	return Steps( _steps[0], _steps[1], _steps[2], _steps[3] );
}
void MotorMoves::steps_from_origin( Steps s ) {
	std::unique_lock<std::mutex> l( m_steps_ );
	for ( int i = 0; i < 4; i++ ) _steps[i] = s[i];
}

void MotorMoves::reset_origin() {
	std::unique_lock<std::mutex> l( m_steps_ );
	_steps[0] = 0;
	_steps[1] = 0;
	_steps[2] = 0;
	_steps[3] = 0;
}

int MotorMoves::get_min_step_time_us() {
	return _minStepTime;
}

std::array<unsigned char,4>  MotorMoves::buttons_state() {
	return _buttons_p->getButtons();
}


void MotorMoves::worker( MotorMoves *pThis ) {
	{
		sched_param sch_params;
		sch_params.sched_priority = sched_get_priority_max ( SCHED_RR ) ;

		if( pthread_setschedparam ( pthread_self (), SCHED_RR, &sch_params ) ) {
			std::cerr << "Warning: Failed to set Thread scheduling : " << std::strerror( errno ) << std::endl;
		}
	}

	auto nt = steady_clock::now();
	auto minimalDelay = std::chrono::microseconds( pThis->_minStepTime );
	auto nnow = steady_clock::now();
	auto delayToGo = minimalDelay; // used in main loop to handle delays after the iteration
	MotorCommand currentCommand; // command extracted in main loop - we don't want to have constructors inside main loop

	auto prevTime = steady_clock::now();

	i_Stepper &_motors = *pThis->getMotors();
	i_Spindle &_spindle = *pThis->getSpindle();
	pThis->paused_ = false;
	while ( true ) {
		if ( pThis->paused_ ) {
			std::this_thread::sleep_until ( steady_clock::now() + std::chrono::microseconds( 200 ) );
			prevTime = steady_clock::now();
		} else {
			currentCommand = pThis->_commands.pop();
			std::unique_lock<std::mutex> lock( pThis->m_steps_ );
			if ( currentCommand.commands == MotorCommand::Command::halt ) {
				break;
			}
			if ( currentCommand.commands != MotorCommand::Command::step ) lock.unlock();
			delayToGo = ( ( currentCommand.delayBefore > pThis->_minStepTime ) || ( currentCommand.commands == MotorCommand::Command::nop ) ) ? std::chrono::microseconds( currentCommand.delayBefore ) : minimalDelay;
			if ( minimalDelay > delayToGo ) delayToGo = minimalDelay;
			nt = prevTime + delayToGo;
			nnow = steady_clock::now();
			if ( nt < nnow ) {
				std::this_thread::sleep_until ( nnow + minimalDelay );
				nt = steady_clock::now();
			} else {
				std::this_thread::sleep_until ( nt );
			}

			prevTime = nt;
			if ( currentCommand.commands == MotorCommand::Command::step ) {
				bool doit = false;
				for ( unsigned  i = 0; i < currentCommand.steps.size(); i++ ) {
					doit = doit | ( currentCommand.steps[i] != 0 );
					pThis->_steps[i] += currentCommand.steps[i];
				}
				if ( doit ) _motors.step( currentCommand.steps );
				lock.unlock();
			} else if ( currentCommand.commands == MotorCommand::Command::steppersOn ) {
				_motors.enabled( true );
			} else if ( currentCommand.commands == MotorCommand::Command::steppersOff ) {
				_motors.enabled( false );
			} else if ( currentCommand.commands == MotorCommand::Command::spindle ) {
				_spindle.setSpeed( ( ( double )currentCommand.steps[0] ) / 127.0 );
			} // else noop
		}

	}

}

MotorMoves::MotorMoves( i_Stepper *motors_, i_Spindle *spindle_, i_Buttons *buttons_,  int minStepTime_ ) : _steps {0, 0, 0, 0}, _commands( 400 ) {
	_motors_p = motors_;
	_spindle_p = spindle_;
	_buttons_p = buttons_;

	_minStepTime = minStepTime_;

	worker_thread_ = std::thread( worker, this );
}

MotorMoves::MotorMoves( p_Stepper motors_p, p_Spindle spindle_p, p_Buttons buttons_p,  int minStepTime_ ) : _steps {0, 0, 0}, _commands( 400 ) {
	_motors_sp = motors_p;
	_spindle_sp = spindle_p;
	_buttons_sp = buttons_p;
	_motors_p = _motors_sp.get();
	_spindle_p = _spindle_sp.get();
	_buttons_p = _buttons_sp.get();
	_minStepTime = minStepTime_;

	worker_thread_ = std::thread( worker, this );
}

MotorMoves::~MotorMoves() {
	_commands.push ( {1, {0, 0, 0}, MotorCommand::Command::halt } );
	worker_thread_.join();
}




std::shared_ptr < i_MotorMoves > MotorMoves_factory( i_Stepper* motors_, i_Spindle * spindle_, i_Buttons *buttons_, int minStepTime_ ) {
	return std::shared_ptr < i_MotorMoves >( new MotorMoves( motors_, spindle_, buttons_, minStepTime_ ) );
}
std::shared_ptr < i_MotorMoves > MotorMoves_factory( p_Stepper motors_, p_Spindle spindle_, p_Buttons buttons_, int minStepTime_ ) {
	return std::shared_ptr < i_MotorMoves >( new MotorMoves( motors_, spindle_, buttons_, minStepTime_ ) );
}

}
}














