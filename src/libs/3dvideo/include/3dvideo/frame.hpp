#pragma once

#include <memory>

#include <util/concurrent_queue.hpp>


class Frame
{
public:
    Frame();
    ~Frame();
};

typedef ConcurrentQueue<std::shared_ptr<Frame>> FrameQueue;
