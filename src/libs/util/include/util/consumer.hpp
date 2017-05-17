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

    virtual void init()
    {
    }

    virtual void run()
    {
        bool hasItems = false;
        while (hasItems || !cancel)
            hasItems = loopBody();
    }

protected:
    bool loopBody()
    {
        Item item;
        const bool hasItem = q.pop(item, timeoutMs);
        if (!hasItem)
            return false;  // timed out

        process(item);
        return true;
    }

    virtual void process(Item &)
    {
    }

protected:
    static constexpr int defaultTimeoutMs = 100;
    int timeoutMs = defaultTimeoutMs;

    QueueType &q;
    CancellationToken &cancel;
};
