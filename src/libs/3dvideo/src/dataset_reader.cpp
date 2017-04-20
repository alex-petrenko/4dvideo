#include <util/tiny_logger.hpp>

#include <3dvideo/app_state.hpp>
#include <3dvideo/dataset_reader.hpp>


DatasetReader::DatasetReader(const std::string &path, const CancellationToken &cancellationToken)
    : FrameProducer(cancellationToken)
    , dataset(path)
{
}

DatasetReader::~DatasetReader()
{
}

void DatasetReader::init()
{
    const auto status = dataset.readHeader();
    if (status != Status::SUCCESS)
    {
        TLOG(ERROR) << "Could not read dataset header!";
        return;
    }

    const auto metadata = dataset.getMetadata();
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

    while (!cancel && !dataset.finished())
    {
        std::shared_ptr<Frame> frame = std::make_shared<Frame>();
        dataset.readFrame(*frame);
        produce(frame);
    }
}
