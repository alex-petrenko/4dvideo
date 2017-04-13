#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include <util/consumer.hpp>
#include <util/concurrent_queue.hpp>


class Frame
{
public:
    Frame(const cv::Mat &color, const cv::Mat &depth);
    ~Frame();

public:
    cv::Mat color, depth;
};

typedef ConcurrentQueue<std::shared_ptr<Frame>> FrameQueue;
typedef Consumer<FrameQueue> FrameConsumer;
