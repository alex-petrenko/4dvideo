#include <thread>

#include <util/tiny_logger.hpp>

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
        Consumer<FrameQueue> writer(writerQueue, cancellationToken);
        writer.run();
    });

    std::thread visualizerThread([&]()
    {
        GrabberVisualizer visualizer(visualizerQueue, cancellationToken);
        visualizer.run();
    });

    grabberThread.join();
    writerThread.join();
    visualizerThread.join();

    return EXIT_SUCCESS;
}
