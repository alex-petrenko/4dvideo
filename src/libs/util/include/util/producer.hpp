#pragma once

#include <util/tiny_logger.hpp>


template<typename QueueType>
class Producer
{
protected:
    typedef typename QueueType::ElementType Item;

public:
    Producer()
    {
    }

    virtual ~Producer()
    {
    }

    virtual void run() = 0;

    virtual void addQueue(QueueType *queue)
    {
        queues.push_back(queue);
    }

protected:
    /// Multiple queues used to pass frames to different consumers.
    std::vector<QueueType *> queues;
};
