#pragma once

#include <3dvideo/frame.hpp>
#include <3dvideo/dataset_output.hpp>


class DatasetWriter : public FrameConsumer
{
public:
    DatasetWriter(FrameQueue &q, CancellationToken &cancel);
    virtual ~DatasetWriter();

    virtual void init();

private:
    virtual void process(std::shared_ptr<Frame> &frame);

private:
    DatasetOutput dataset;
};
