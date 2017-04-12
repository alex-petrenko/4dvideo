#pragma once

#include <util/tiny_logger.hpp>


template<typename QueueType>
class Consumer
{
    typedef typename QueueType::ElementType Item;

public:
    Consumer(QueueType &q)
        : q(q)
    {
    }

    void run()
    {
        while (true)
        {
            // cancellation token?
            Item item;
            const bool hasItem = q.pop(item, timeoutMs);
            if (!hasItem)
                continue;  // timed out

            process(item);
        }
    }

private:
    void process(Item &item)
    {
        TLOG(INFO);
    }

private:
    static constexpr int defaultTimeoutMs = 300;

    int timeoutMs = defaultTimeoutMs;
    QueueType &q;
};
