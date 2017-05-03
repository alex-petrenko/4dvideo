#include <util/tiny_logger.hpp>

#include <3dvideo/app_state.hpp>
#include <3dvideo/dataset_reader.hpp>


DatasetReader::DatasetReader(const std::string &path, const CancellationToken &cancellationToken)
    : FrameProducer(cancellationToken)
    , path(path)
    , dataset(std::make_shared<DatasetInput>(path))
{
}

DatasetReader::~DatasetReader()
{
}

void DatasetReader::init()
{
    const auto status = dataset->readHeader();
    if (status != Status::SUCCESS)
    {
        TLOG(ERROR) << "Could not read dataset header!";
        return;
    }

    const auto metadata = dataset->getMetadata();
    auto &sensorManager = appState().getSensorManager();
    sensorManager.setColorParams(metadata.color, metadata.colorFormat);
    sensorManager.setDepthParams(metadata.depth, metadata.depthFormat);
    sensorManager.setInitialized();
    initialized = true;
}

void DatasetReader::run()
{
    if (!initialized)
        return;

    int numFrames = 0;
    while (!cancel && !dataset->finished())
    {
        std::shared_ptr<Frame> frame = std::make_shared<Frame>();
        if (dataset->readFrame(*frame) == Status::SUCCESS)
        {
            produce(frame);
            ++numFrames;
        }
        else
            TLOG(ERROR) << "Error while reading frame!";
    }

    TLOG(INFO) << "Read total: " << numFrames << " frames";
}

void DatasetReader::runLoop()
{
    while (!cancel)
    {
        run();

        initialized = false;
        dataset = std::make_shared<DatasetInput>(path);
        init();
    }
}
