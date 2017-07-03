#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/mesher.hpp>
#include <3dvideo/player.hpp>
#include <3dvideo/depth_filter.hpp>
#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/animation_writer.hpp>


int main(int argc, char *argv[])
{
    const int minNumArgs = 3;
    if (argc < minNumArgs)
        TLOG(FATAL) << "Expected at least" << minNumArgs << " arguments, got " << argc;

    int arg = 1;
    const std::string datasetPath(argv[arg++]), outputPath(argv[arg++]);

    CancellationToken cancellationToken;
    FrameQueue frameQueue(100), filteredDepthQueue(100);
    MeshFrameQueue playerQueue(10), writerQueue(200);

    std::thread readerThread([&]
    {
        DatasetReader reader(datasetPath, true, cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.run();
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
        meshFrameProducer.addQueue(&writerQueue);

        Mesher mesher(filteredDepthQueue, meshFrameProducer, cancellationToken);
        mesher.init();
        mesher.run();
    });

    std::thread writerThread([&]
    {
        AnimationWriter writer(outputPath, writerQueue, cancellationToken);
        writer.init();
        writer.run();
    });

    Player player(playerQueue, cancellationToken);
    player.init();
    player.run();

    cancellationToken.trigger();

    readerThread.join();
    filterThread.join();
    mesherThread.join();
    writerThread.join();

    return EXIT_SUCCESS;
}
