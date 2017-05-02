#include <chrono>
#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/data_visualizer.hpp>


using namespace std::chrono_literals;


class WriterCvt : public DatasetWriter
{
public:
    WriterCvt(const std::string &path, FrameQueue &q, CancellationToken &cancel)
        :DatasetWriter(path, q, cancel)
    {
    }

    virtual ~WriterCvt()
    {}

protected:
    virtual void process(std::shared_ptr<Frame> &frame) override
    {
        TLOG(INFO) << frame->dTimestamp;
        DatasetWriter::process(frame);
    }
};


int main()
{
    CancellationToken cancellationToken;
    FrameQueue frameQueue;

    std::thread readerThread([&]
    {
        DatasetReader reader(R"(C:\all\projects\itseez\data\testing\special_datasets\002_yoga_wall.4dv)", cancellationToken);
        // DatasetReader reader(R"(C:\temp\tst\dataset.4dv)", cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.run();
    });

    WriterCvt writer(R"(C:\all\projects\itseez\data\testing\special_datasets\002_yoga_wall_cvt.4dv)", frameQueue, cancellationToken);
    writer.init();
    writer.run();

    readerThread.join();

    return EXIT_SUCCESS;
}
