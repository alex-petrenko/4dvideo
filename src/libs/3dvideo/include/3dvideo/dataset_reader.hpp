#pragma once

#include <3dvideo/frame.hpp>
#include <3dvideo/dataset_input.hpp>


class DatasetReader : public FrameProducer
{
public:
    DatasetReader(const std::string &path, const CancellationToken &cancellationToken);
    virtual ~DatasetReader();

    void init();

    virtual void run();

private:
    DatasetInput dataset;
};
