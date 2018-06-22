#ifndef __TP_MOTOR_SAFEQUEUE_HPP__
#define __TP_MOTOR_SAFEQUEUE_HPP__
#include <queue>
#include <mutex>
#include <condition_variable>

namespace tp {
namespace motor {
    // the SafeQueue was described inarticle https://juanchopanzacpp.wordpress.com/2013/02/26/concurrent-queue-c11/
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
