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
    void put(T &&item)
    {
        std::unique_lock<std::mutex> lock(mutex);
        q.emplace(item);

        lock.unlock();  // should unlock before notify, otherwise waiting thread will wake up only to block again
        cv.notify_one();
    }

    bool pop(T &item, int timeoutMs)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (q.empty())
        {
            cv.wait_for(lock, std::chrono::milliseconds(timeoutMs));
            if (q.empty())
                return false;
        }

        item = q.front();
        q.pop();
        return true;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return q.empty();
    }

private:
    std::queue<T> q;
    mutable std::mutex mutex;
    std::condition_variable cv;
};
