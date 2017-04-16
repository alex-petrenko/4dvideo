#pragma once

#include <util/cancellation_token.hpp>


template<typename QueueType>
class Consumer
{
protected:
    typedef typename QueueType::ElementType Item;

public:
    Consumer(QueueType &q, CancellationToken &cancel)
        : q(q)
        , cancel(cancel)
    {
    }

    virtual ~Consumer() = default;

    void run()
    {
        while (!cancel)
        {
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
    }

protected:
    CancellationToken &cancel;

private:
    static constexpr int defaultTimeoutMs = 100;

    int timeoutMs = defaultTimeoutMs;

    QueueType &q;
};
