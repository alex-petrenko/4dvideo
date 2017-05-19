#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/mesher.hpp>
#include <3dvideo/player.hpp>
#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/animation_writer.hpp>


int main(int argc, char *argv[])
{
    const int minNumArgs = 2;
    if (argc < minNumArgs)
    {
        TLOG(ERROR) << "Expected at least" << minNumArgs << " arguments, got " << argc;
        return EXIT_FAILURE;
    }

    int arg = 1;
    const std::string datasetPath(argv[arg++]);
    std::string outputPath;
    if (argc > arg)
        outputPath = argv[arg++];

    CancellationToken cancellationToken;
    FrameQueue frameQueue(100);
    MeshFrameQueue playerQueue(10), writerQueue(200);

    std::thread readerThread([&]
    {
        DatasetReader reader(datasetPath, cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.run();
    });

    std::thread mesherThread([&]
    {
        MeshFrameProducer meshFrameProducer(cancellationToken);
        meshFrameProducer.addQueue(&playerQueue);
        meshFrameProducer.addQueue(&writerQueue);

        Mesher mesher(frameQueue, meshFrameProducer, cancellationToken);
        mesher.init();
        mesher.run();
    });

    std::thread writerThread([&]
    {
        if (outputPath.empty())
            return;

        AnimationWriter writer(outputPath, writerQueue, cancellationToken);
        writer.init();
        writer.run();
    });

    Player player(playerQueue, cancellationToken);
    player.init();
    player.run();

    cancellationToken.trigger();

    readerThread.join();
    mesherThread.join();
    writerThread.join();

    return EXIT_SUCCESS;
}
