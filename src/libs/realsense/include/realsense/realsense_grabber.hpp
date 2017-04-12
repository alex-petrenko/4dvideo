#pragma once

#include <memory>

#include <3dvideo/frame.hpp>


class RealsenseGrabber
{
    /// Private implementation to hide some realsense headers.
    struct RealsenseGrabberImpl;

public:
    RealsenseGrabber();
    ~RealsenseGrabber();

    void init();
    void run();

    void addQueue(FrameQueue *queue);

private:
    std::unique_ptr<RealsenseGrabberImpl> data;
};
