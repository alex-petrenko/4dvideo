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
    void put(T item)
    {
        std::unique_lock<std::mutex> lock(mutex);
        q.push(item);
        notify(lock);
    }

    void put(T &&item)
    {
        std::unique_lock<std::mutex> lock(mutex);
        q.emplace(item);
        notify(lock);
    }

    /// Returns false if there are no items in the queue after timeoutMs milliseconds.
    bool pop(T &item, int timeoutMs)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (q.empty())
            cv.wait_for(lock, std::chrono::milliseconds(timeoutMs));

        if (q.empty())
            return false;  // timed out

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
    void notify(std::unique_lock<std::mutex> &lock)
    {
        lock.unlock();  // should unlock before notify, otherwise waiting thread will wake up only to block again
        cv.notify_one();
    }

private:
    std::queue<T> q;
    mutable std::mutex mutex;
    std::condition_variable cv;
};
