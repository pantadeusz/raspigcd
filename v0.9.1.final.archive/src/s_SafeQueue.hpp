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


    The SafeQueue was described in article https://juanchopanzacpp.wordpress.com/2013/02/26/concurrent-queue-c11/
    The most of the code is taken from there, but I made some major updates to make it work with the rest of my code

*/

#ifndef __TP_MOTOR_SAFEQUEUE_HPP__
#define __TP_MOTOR_SAFEQUEUE_HPP__
#include <queue>
#include <mutex>
#include <condition_variable>

namespace tp {
namespace motor {
    template <typename T>
    class SafeQueue {
    private:
        std::vector<T> ring_;
        int head_, tail_;
        std::mutex m_;
        std::condition_variable c_;
        int maxSize_;

    public:
        inline void push(const T& e)
        {
            std::unique_lock<std::mutex> l(m_);
            while ((int)((maxSize_ + head_ - tail_) % maxSize_) >= (maxSize_ - 1)) {
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
            while (head_ == tail_) {
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
            while (head_ == tail_) {
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
            while (head_ != tail_) {
                c_.wait(l);
            }
            c_.notify_one();
        }

        SafeQueue(int maxSize)
            : ring_(maxSize)
            , maxSize_(maxSize)
        {
            head_ = tail_ = 0;
        }

        SafeQueue& operator=(const SafeQueue&) = delete;
        SafeQueue(const SafeQueue&) = delete;
    };

} // namespace motor
} // namespace tp
#endif
