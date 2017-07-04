#pragma once

#include <4d/frame.hpp>


class DepthFilter : public FrameConsumer
{
public:
    DepthFilter(FrameQueue &inputQueue, FrameProducer &output, CancellationToken &cancellationToken);

    void init() override;

protected:
    void process(std::shared_ptr<Frame> &frame) override;

private:
    constexpr static bool skipFiltering = false;

    FrameProducer &output;
    Calibration calibration;
    CameraParams depthCam, colorCam;
};
