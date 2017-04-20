#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include <util/consumer.hpp>
#include <util/producer.hpp>
#include <util/concurrent_queue.hpp>


class Frame
{
public:
    Frame();
    Frame(int idx, const cv::Mat &color, int64_t cTimestamp, const cv::Mat &depth, int64_t dTimestamp);
    ~Frame();

public:
    int frameNumber = 0;
    cv::Mat color, depth;
    int64_t cTimestamp = 0, dTimestamp = 0;
};

typedef ConcurrentQueue<std::shared_ptr<Frame>> FrameQueue;
typedef Producer<FrameQueue> FrameProducer;
typedef Consumer<FrameQueue> FrameConsumer;
