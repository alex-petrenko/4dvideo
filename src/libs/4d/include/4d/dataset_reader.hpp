#pragma once

#include <4d/frame.hpp>
#include <4d/dataset_input.hpp>


class DatasetReader : public FrameProducer
{
public:
    DatasetReader(const std::string &path, bool readColor, const CancellationToken &cancellationToken);
    virtual ~DatasetReader();

    void init();

    virtual void run();

    /// Read dataset in a loop
    virtual void runLoop();

private:
    bool withColor;
    bool initialized = false;
    std::string path;
    std::shared_ptr<DatasetInput> dataset;
};
