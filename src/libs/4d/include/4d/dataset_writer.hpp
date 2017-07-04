#pragma once

#include <4d/frame.hpp>
#include <4d/dataset_output.hpp>


class DatasetWriter : public FrameConsumer
{
public:
    DatasetWriter(const std::string &path, FrameQueue &q, CancellationToken &cancel);
    virtual ~DatasetWriter() override;

    virtual void init() override;

protected:
    virtual void process(std::shared_ptr<Frame> &frame) override;

private:
    DatasetOutput dataset;
};
