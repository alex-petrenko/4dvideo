#pragma once


#include <util/cancellation_token.hpp>


template<typename QueueType>
class Producer
{
protected:
    typedef typename QueueType::ElementType Item;

public:
    Producer(const CancellationToken &cancel)
        : cancel(cancel)
    {
    }

    virtual ~Producer()
    {
    }

    virtual void addQueue(QueueType *queue)
    {
        queues.push_back(queue);
    }

    virtual void produce(Item item)
    {
        for (auto queue : queues)
            while (!cancel && !queue->put(item, 100));
    }

protected:
    /// Multiple queues used to pass frames to different consumers.
    std::vector<QueueType *> queues;
    const CancellationToken &cancel;
};
