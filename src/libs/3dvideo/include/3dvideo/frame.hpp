#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include <util/concurrent_queue.hpp>


class Frame
{
public:
    Frame(const cv::Mat &color);
    ~Frame();

public:
    cv::Mat color;
};

typedef ConcurrentQueue<std::shared_ptr<Frame>> FrameQueue;
