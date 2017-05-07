#include <chrono>
#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/data_visualizer.hpp>


using namespace std::chrono_literals;


int main()
{
    CancellationToken cancellationToken;
    FrameQueue frameQueue;

    std::thread readerThread([&]
    {
        DatasetReader reader(R"(C:\all\projects\itseez\data\testing\special_datasets\004_first_realsense.4dv)", cancellationToken);
        // DatasetReader reader(R"(C:\temp\tst\dataset.4dv)", cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.run();
        cancellationToken.trigger();
    });

    DatasetWriter writer(R"(C:\all\projects\itseez\data\testing\special_datasets\004_first_realsense_cvt.4dv)", frameQueue, cancellationToken);
    writer.init();
    writer.run();

    readerThread.join();

    return EXIT_SUCCESS;
}
