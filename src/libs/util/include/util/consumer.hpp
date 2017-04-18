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
        while (!cancel)
            loopBody();
    }

protected:
    void loopBody()
    {
        Item item;
        const bool hasItem = q.pop(item, timeoutMs);
        if (!hasItem)
            return;  // timed out

        process(item);
    }

private:
    virtual void process(Item &)
    {
    }

protected:
    static constexpr int defaultTimeoutMs = 100;
    int timeoutMs = defaultTimeoutMs;

    QueueType &q;
    CancellationToken &cancel;
};
