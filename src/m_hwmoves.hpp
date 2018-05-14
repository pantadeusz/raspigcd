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

#ifndef ___MOVEMENT_HPP____
#define ___MOVEMENT_HPP____

#include "m_motor_interface.hpp"

#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <array>

#include <chrono>
#include <thread>

namespace tp
{
namespace motor
{

// kroki
class Steps : public std::array<int, 4>
{
  public:
	Steps(const int a, const int b, const int c) : std::array<int, 4>{a, b, c, 0} {}
	Steps(const int a, const int b, const int c, const int d) : std::array<int, 4>{a, b, c, d} {}
	Steps() : std::array<int, 4>{0, 0, 0, 0} {}
};

inline Steps operator+(const Steps &a, const Steps &b)
{
	return Steps(a[0] + b[0], a[1] + b[1], a[2] + b[2],a[3] + b[3]);
}
inline Steps operator-(const Steps &a, const Steps &b)
{
	return Steps(a[0] - b[0], a[1] - b[1], a[2] - b[2],a[3] - b[3] );
}
inline Steps operator*(const Steps &a, const Steps &b)
{
	return Steps(a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]);
}
inline Steps operator*(const Steps &a, const int &b)
{
	return Steps(a[0] * b, a[1] * b, a[2] * b, a[3] * b);
}
inline Steps operator/(const Steps &a, const int &b)
{
	return Steps(a[0] / b, a[1] / b, a[2] / b, a[3] / b);
}
inline Steps operator/(const Steps &a, const Steps &b)
{
	return Steps(a[0] / b[0], a[1] / b[1], a[2] / b[2], a[3] / b[3]);
}
inline double len2(Steps &s)
{
	return s[0] * s[0] + s[1] * s[1] + s[2] * s[2] + s[3] * s[3];
}
inline double len2(Steps &s, bool x, bool y, bool z)
{
	return (x ? s[0] * s[0] : 0) + (y ? s[1] * s[1] : 0) + (z ? s[2] * s[2] : 0);
}

// the SafeQueue was based on article https://juanchopanzacpp.wordpress.com/2013/02/26/concurrent-queue-c11/
template <typename T>
class SafeQueue
{
  private:
	std::vector<T> ring_;
	int head_, tail_;
	std::mutex m_;
	std::condition_variable c_;
	int maxSize_;

  public:
	inline void push(const T &e)
	{
		std::unique_lock<std::mutex> l(m_);
		while ((int)((maxSize_ + head_ - tail_) % maxSize_) >= (maxSize_ - 1))
		{
			c_.wait(l);
		}
		ring_[head_] = e;
		head_ = (head_ + 1) % maxSize_;
		l.unlock();
		c_.notify_one();
	}
	inline T pop()
	{
		std::unique_lock<std::mutex> l(m_);
		while (head_ == tail_)
		{
			c_.wait(l);
		}
		T v = ring_[tail_];
		tail_ = (tail_ + 1) % maxSize_;
		l.unlock();
		c_.notify_one();
		return v;
	}
	inline T peek()
	{
		std::unique_lock<std::mutex> l(m_);
		while (head_ == tail_)
		{
			c_.wait(l);
		}
		T v = ring_[tail_];
		l.unlock();
		c_.notify_one();
		return v;
	}
	inline bool empty()
	{
		std::unique_lock<std::mutex> mlock(m_);
		return head_ == tail_;
	}
	inline void removeAll()
	{
		std::unique_lock<std::mutex> mlock(m_);
		head_ = tail_ = 0;
		c_.notify_all();
	}
	inline void waitEmpty()
	{
		std::unique_lock<std::mutex> l(m_);
		while (head_ != tail_)
		{
			c_.wait(l);
		}
		c_.notify_one();
	}

	SafeQueue(int maxSize) : ring_(maxSize), maxSize_(maxSize)
	{
		head_ = tail_ = 0;
	}

	SafeQueue &operator=(const SafeQueue &) = delete;
	SafeQueue(const SafeQueue &) = delete;
};

class MotorCommand
{
  public:
	enum Command
	{
		step,
		steppersOn,
		steppersOff,
		nop,
		halt,
		spindle
	};
	int delayBefore;
	std::array<signed char, 3> steps; // steps to perform
	char commands;					  // additional commands - turn on or off motors
};

class i_MotorMoves
{
  protected:
  public:
	/*
	 * pushes one command to command queue
	 */
	virtual void push(const MotorCommand &command) = 0;
	/**
	 * waits until command queue is empty. WARNING: If queue is paused, then the behavior of this
	 * method is unspecified!
	 */
	virtual void wait_finished() = 0;
	/**
	 * returns number of steps from origin
	 */
	virtual Steps steps_from_origin() = 0;

	/**
	 * sets steps from origin
	 */
	virtual void steps_from_origin(Steps s) = 0;

	virtual std::array<unsigned char, 4> buttons_state() = 0;

	virtual bool is_steppers_enabled() = 0;

	virtual void clear_command_queue() = 0;
	/**
	 * resets origin to 0
	 */
	virtual void reset_origin() = 0;

	virtual int get_min_step_time_us() = 0;

	/**
	 * blocks or unblocks reading command queue.
	 * */
	virtual void set_paused(bool paused) = 0;

	/**
	 * checks if the queue is paused.
	 * */
	virtual bool is_paused() = 0;
};

std::shared_ptr<i_MotorMoves> MotorMoves_factory(i_Stepper *motors_, i_Spindle *spindle_, i_Buttons *buttons_, int minStepTime_);
std::shared_ptr<i_MotorMoves> MotorMoves_factory(p_Stepper motors_, p_Spindle spindle_, p_Buttons buttons_, int minStepTime_);

} // namespace motor
} // namespace tp

#endif
