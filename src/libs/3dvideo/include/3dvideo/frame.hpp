#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include <util/consumer.hpp>
#include <util/producer.hpp>
#include <util/concurrent_queue.hpp>


class Frame
{
public:
    Frame(int idx, const cv::Mat &color, const cv::Mat &depth);
    ~Frame();

public:
    int frameNumber = 0;
    cv::Mat color, depth;
};

typedef ConcurrentQueue<std::shared_ptr<Frame>> FrameQueue;
typedef Producer<FrameQueue> FrameProducer;
typedef Consumer<FrameQueue> FrameConsumer;
