#pragma once

#include <memory>

#include <util/producer.hpp>

#include <4d/frame.hpp>


class RealsenseGrabber : public FrameProducer
{
    /// Private implementation to hide some realsense headers.
    struct RealsenseGrabberImpl;

public:
    RealsenseGrabber(const CancellationToken &cancellationToken);
    virtual ~RealsenseGrabber();

    void init();

    virtual void run();

private:
    std::unique_ptr<RealsenseGrabberImpl> data;
};
