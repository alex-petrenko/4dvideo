#pragma once

#include <3dvideo/frame.hpp>


class DepthFilter : public FrameConsumer
{
public:
    DepthFilter(FrameQueue &inputQueue, FrameProducer &output, CancellationToken &cancellationToken);

protected:
    void process(std::shared_ptr<Frame> &frame) override;

private:
    FrameProducer &output;
};
