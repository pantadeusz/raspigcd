#ifndef ___QUEUE_T_PUZNIAKOWSKI___
#define ___QUEUE_T_PUZNIAKOWSKI___
#include <atomic>
#include <vector>
#include <list>

namespace tp {


template <class T>
class fifo_c
{
    std::atomic_flag lock;
    std::list<T> data;

public:
    fifo_c<T>() : lock(ATOMIC_FLAG_INIT){};
    T get(std::atomic<bool>& cancel_execution)
    {
        while (!cancel_execution) {
            while (lock.test_and_set(std::memory_order_acquire))
                ;
            if (data.size() > 0) {
                auto ret = data.front();
                data.pop_front();
                lock.clear(std::memory_order_release);
                return ret;
            } else {
                lock.clear(std::memory_order_release);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        throw std::invalid_argument("fifo_c: the get from front broken.");
    }

    void put(std::atomic<bool>& cancel_execution, T value, int max_queue_size = 3)
    {
        while (!cancel_execution) {
            while (lock.test_and_set(std::memory_order_acquire))
                ;
            if ((int)data.size() < (int)max_queue_size) {
                data.push_back(value);
                lock.clear(std::memory_order_release);
                return;
            } else {
                lock.clear(std::memory_order_release);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }

        throw std::invalid_argument("fifo_c: the put method broken.");
    }

    size_t size()
    {
        size_t ret;
        while (lock.test_and_set(std::memory_order_acquire))
            ;
        ret = data.size();
        lock.clear(std::memory_order_release);
        return ret;
    }
};

} // namespace tp
#endif