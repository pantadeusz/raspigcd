#ifndef ___QUEUE_T_PUZNIAKOWSKI___
#define ___QUEUE_T_PUZNIAKOWSKI___
#include <atomic>
#include <stdexcept>
#include <vector>

namespace raspigcd {


template <class T>
class fifo_c
{
    std::atomic_flag lock;
    std::vector<T> data;
    unsigned int l, s, max_queue_size; ///< pointer to the first elememtn, size of queue, maximal size


public:
    fifo_c<T>(const unsigned int size = 32000) : lock(ATOMIC_FLAG_INIT)
    {
        if (size <= 0) throw std::invalid_argument("the queue size must be greater than 0");
        l = s = 0;
        max_queue_size = size;
        data.resize(max_queue_size);
    };
    inline T get(std::atomic<bool>& cancel_execution)
    {
        while (!cancel_execution) {
            while (lock.test_and_set(std::memory_order_acquire))
                ;
            if (s > 0) {
                auto ret = data[l];
                l = (l + 1) % max_queue_size;
                s--;
                lock.clear(std::memory_order_release);
                return ret;
            } else {
                lock.clear(std::memory_order_release);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        throw std::invalid_argument("fifo_c: the get from front broken.");
    }

    void put(std::atomic<bool>& cancel_execution, const T value)
    {
        while (!cancel_execution) {
            while (lock.test_and_set(std::memory_order_acquire))
                ;
            if (s < max_queue_size) {
                data[(l + s) % max_queue_size] = value;
                s++;
                lock.clear(std::memory_order_release);
                return;
            } else {
                lock.clear(std::memory_order_release);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        throw std::invalid_argument("fifo_c: the put method broken.");
    }

    /**
     * returns the buffer size - how many values are in the buffer
     * */
    size_t size()
    {
        while (lock.test_and_set(std::memory_order_acquire))
                ;
        auto v = s;
        lock.clear(std::memory_order_release);
        return v;
    }
};

} // namespace raspigcd
#endif
