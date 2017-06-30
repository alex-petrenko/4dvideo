#pragma once

#include <string>

#include <3dvideo/frame.hpp>


class DataVisualizer : public FrameConsumer
{
    static_assert(std::is_same<std::shared_ptr<Frame>, FrameConsumer::Item>::value, "Consumer item type must be Frame ptr");

public:
    DataVisualizer(FrameQueue &q, CancellationToken &cancellationToken);
    virtual ~DataVisualizer() override;

    virtual void init() override;
    virtual void run() override;

private:
    virtual void process(std::shared_ptr<Frame> &frame);

private:
    CameraParams colorCamera, depthCamera;
    int numFrames = 0;

    static constexpr char *windowName = "Sensor realtime data";
    static constexpr bool saveToDisk = false;
};
