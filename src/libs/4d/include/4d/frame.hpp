#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include <util/consumer.hpp>
#include <util/producer.hpp>
#include <util/concurrent_queue.hpp>


struct Frame
{
    int frameNumber = 0;
    cv::Mat color, depth;
    std::vector<cv::Point3f> cloud;
    int64_t cTimestamp = 0, dTimestamp = 0;
};

typedef ConcurrentQueue<std::shared_ptr<Frame>> FrameQueue;
typedef Producer<FrameQueue> FrameProducer;
typedef Consumer<FrameQueue> FrameConsumer;
