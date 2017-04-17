#pragma once

#include <string>

#include <3dvideo/frame.hpp>


class GrabberVisualizer : public FrameConsumer
{
    static_assert(std::is_same<std::shared_ptr<Frame>, FrameConsumer::Item>::value, "Consumer item type must be Frame ptr");

public:
    GrabberVisualizer(FrameQueue &q, CancellationToken &cancellationToken);
    virtual ~GrabberVisualizer();

    virtual void run();

private:
    virtual void process(std::shared_ptr<Frame> &frame);

private:
    static constexpr char *windowName = "Sensor realtime data";
};
