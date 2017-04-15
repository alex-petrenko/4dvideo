#pragma once

#include <memory>

#include <util/producer.hpp>

#include <3dvideo/frame.hpp>


class RealsenseGrabber : public Producer<FrameQueue>
{
    /// Private implementation to hide some realsense headers.
    struct RealsenseGrabberImpl;

public:
    RealsenseGrabber();
    virtual ~RealsenseGrabber();

    void init();

    virtual void run();

private:
    std::unique_ptr<RealsenseGrabberImpl> data;
};
