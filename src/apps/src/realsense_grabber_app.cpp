#include <thread>
#include <iomanip>

#include <util/tiny_logger.hpp>
#include <util/filesystem_utils.hpp>

#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/data_visualizer.hpp>

#include <realsense/realsense_grabber.hpp>


int main(int argc, char *argv[])
{
    const int numArgs = 2;
    if (argc != numArgs)
        TLOG(FATAL) << "Expected " << numArgs << " arguments, got " << argc;

    int arg = 1;
    const std::string originalPath(argv[arg++]);
    std::string datasetPath(originalPath);

    int fileIdx = 0;
    while (fileExists(datasetPath))
    {
        ++fileIdx;
        std::ostringstream s;
        s << originalPath << std::setw(2) << std::setfill('0') << fileIdx;
        datasetPath = s.str();
    }

    TLOG(INFO) << "Dataset path: " << datasetPath;

    CancellationToken cancellationToken;
    FrameQueue writerQueue, visualizerQueue;

    std::thread grabberThread([&]
    {
        RealsenseGrabber grabber(cancellationToken);
        grabber.addQueue(&writerQueue);
        grabber.addQueue(&visualizerQueue);
        grabber.init();
        grabber.run();
    });

    std::thread writerThread([&]()
    {
        DatasetWriter writer(datasetPath, writerQueue, cancellationToken);
        writer.init();
        writer.run();
    });

    // visualizer works with OpenCV GUI, so it's better to keep it on main thread
    DataVisualizer visualizer(visualizerQueue, cancellationToken);
    visualizer.init();
    visualizer.run();

    writerThread.join();
    grabberThread.join();

    return EXIT_SUCCESS;
}
