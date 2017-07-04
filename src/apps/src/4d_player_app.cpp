#include <thread>

#include <util/tiny_logger.hpp>

#include <4d/mesher.hpp>
#include <4d/player.hpp>
#include <4d/depth_filter.hpp>
#include <4d/dataset_reader.hpp>


int main(int argc, char *argv[])
{
    const int minNumArgs = 2;
    if (argc < minNumArgs)
        TLOG(FATAL) << "Expected at least" << minNumArgs << " arguments, got " << argc;

    int arg = 1;
    const std::string datasetPath(argv[arg++]);

    CancellationToken cancellationToken;
    FrameQueue frameQueue(100), filteredDepthQueue(100);
    MeshFrameQueue playerQueue(10);

    std::thread readerThread([&]
    {
        DatasetReader reader(datasetPath, true, cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.runLoop();
    });

    std::thread filterThread([&]
    {
        FrameProducer filteredDepthProducer(cancellationToken);
        filteredDepthProducer.addQueue(&filteredDepthQueue);

        DepthFilter filter(frameQueue, filteredDepthProducer, cancellationToken);
        filter.init();
        filter.run();
    });

    std::thread mesherThread([&]
    {
        MeshFrameProducer meshFrameProducer(cancellationToken);
        meshFrameProducer.addQueue(&playerQueue);
        Mesher mesher(filteredDepthQueue, meshFrameProducer, cancellationToken);
        mesher.init();
        mesher.run();
    });

    Player player(playerQueue, cancellationToken);
    player.init();
    player.run();

    cancellationToken.trigger();
    readerThread.join(), filterThread.join(), mesherThread.join();

    return EXIT_SUCCESS;
}
