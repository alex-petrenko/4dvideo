#include <thread>

#include <opencv2/highgui.hpp>

#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/grabber_visualizer.hpp>

#include <realsense/realsense_grabber.hpp>


int main()
{
    CancellationToken cancellationToken;
    FrameQueue writerQueue, visualizerQueue;

    std::thread grabberThread([&]()
    {
        RealsenseGrabber grabber(cancellationToken);
        grabber.addQueue(&writerQueue);
        grabber.addQueue(&visualizerQueue);
        grabber.init();
        grabber.run();
    });

    std::thread writerThread([&]()
    {
        DatasetWriter writer(writerQueue, cancellationToken);
        writer.run();
    });

    // visualizer works with OpenCV GUI, so it's better to keep it on main thread
    GrabberVisualizer visualizer(visualizerQueue, cancellationToken);
    visualizer.run();

    writerThread.join();
    grabberThread.join();

    return EXIT_SUCCESS;
}
