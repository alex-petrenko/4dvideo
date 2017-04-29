#include <thread>

#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/data_visualizer.hpp>


int main()
{
    CancellationToken cancellationToken;
    FrameQueue frameQueue;

    std::thread readerThread([&]
    {
        DatasetReader reader(R"(C:\all\projects\itseez\data\testing\special_datasets\004_first_realsense.4dv)", cancellationToken);
        //DatasetReader reader(R"(C:\temp\tst\dataset.4dv)", cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.run();
    });

    // visualizer works with OpenCV GUI, so it's better to keep it on main thread
    DataVisualizer visualizer(frameQueue, cancellationToken);
    visualizer.run();

    readerThread.join();

    return EXIT_SUCCESS;
}
