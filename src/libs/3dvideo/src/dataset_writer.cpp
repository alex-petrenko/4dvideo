#include <util/tiny_logger.hpp>

#include <3dvideo/app_state.hpp>
#include <3dvideo/dataset_writer.hpp>


using namespace std::chrono_literals;


DatasetWriter::DatasetWriter(const std::string &path, FrameQueue &q, CancellationToken &cancel)
    : Consumer(q, cancel)
    , dataset(path)
{
}

DatasetWriter::~DatasetWriter()
{
}

void DatasetWriter::init()
{
    const SensorManager &sensorManager = appState().getSensorManager();

    while (!cancel && !sensorManager.isInitialized())
        std::this_thread::sleep_for(30ms);

    dataset.writeHeader(sensorManager);
}

void DatasetWriter::process(std::shared_ptr<Frame> &frame)
{
    dataset.writeFrame(*frame);
    TLOG(INFO) << "Finished writing frame #" << frame->frameNumber;
}
