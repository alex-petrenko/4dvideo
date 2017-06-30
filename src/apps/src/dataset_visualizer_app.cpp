#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/data_visualizer.hpp>


int main(int argc, char *argv[])
{
    const int minNumArgs = 2;
    if (argc < minNumArgs)
        TLOG(FATAL) << "Expected at least" << minNumArgs << " arguments, got " << argc;

    int arg = 1;
    const std::string datasetPath(argv[arg++]);

    CancellationToken cancellationToken;
    FrameQueue frameQueue;

    std::thread readerThread([&]
    {
        DatasetReader reader(datasetPath, cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.run();
    });

    // visualizer works with OpenCV GUI, so it's better to keep it on main thread
    DataVisualizer visualizer(frameQueue, cancellationToken);
    visualizer.init();
    visualizer.run();

    readerThread.join();

    return EXIT_SUCCESS;
}
