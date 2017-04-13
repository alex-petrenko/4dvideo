#pragma once

#include <util/tiny_logger.hpp>


template<typename QueueType>
class Consumer
{
protected:
    typedef typename QueueType::ElementType Item;

public:
    Consumer(QueueType &q)
        : q(q)
    {
    }

    virtual ~Consumer() = default;

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
    virtual void process(Item &)
    {
        TLOG(INFO);
    }

private:
    static constexpr int defaultTimeoutMs = 300;

    int timeoutMs = defaultTimeoutMs;
    QueueType &q;
};
