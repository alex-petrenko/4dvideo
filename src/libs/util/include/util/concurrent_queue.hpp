#pragma once

#include <mutex>
#include <queue>
#include <chrono>
#include <condition_variable>


template<typename T>
class ConcurrentQueue
{
public:
    typedef typename T ElementType;

public:
    ConcurrentQueue()
    {
    }

    ConcurrentQueue(size_t maxCapacity)
        : maxCapacity(maxCapacity)
    {
    }

    void setMaxCapacity(size_t capacity)
    {
        std::lock_guard<std::mutex> lock(mutex);
        maxCapacity = capacity;
    }

    /// Returns false if there's no available space in the buffer after timeoutMs milliseconds.
    bool put(T item, int timeoutMs = std::numeric_limits<size_t>::max())
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (q.size() >= maxCapacity)
            capacityCV.wait_for(lock, std::chrono::milliseconds(timeoutMs));

        if (q.size() >= maxCapacity)
            return false;  // timed out

        q.push(std::move(item));
        notify(lock, newItemCV);
        return true;
    }

    /// Returns false if there are no items in the queue after timeoutMs milliseconds.
    bool pop(T &item, int timeoutMs)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (q.empty())
            newItemCV.wait_for(lock, std::chrono::milliseconds(timeoutMs));

        if (q.empty())
            return false;  // timed out

        item = q.front();
        q.pop();
        notify(lock, capacityCV);
        return true;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return q.empty();
    }

private:
    void notify(std::unique_lock<std::mutex> &lock, std::condition_variable &cv)
    {
        lock.unlock();  // should unlock before notify, otherwise waiting thread will wake up only to block again
        cv.notify_one();
    }

private:
    size_t maxCapacity = std::numeric_limits<size_t>::max();
    std::queue<T> q;
    mutable std::mutex mutex;
    std::condition_variable newItemCV, capacityCV;
};
