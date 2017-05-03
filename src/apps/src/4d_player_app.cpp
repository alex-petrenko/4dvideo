#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/player.hpp>
#include <3dvideo/dataset_reader.hpp>


int main(int argc, char *argv[])
{
    const int numArgs = 2;
    if (argc != numArgs)
    {
        TLOG(ERROR) << "Expected " << numArgs << " arguments, got " << argc;
        return EXIT_FAILURE;
    }

    int arg = 1;
    const std::string datasetPath(argv[arg++]);

    CancellationToken cancellationToken;
    FrameQueue frameQueue(100);

    std::thread readerThread([&]
    {
        DatasetReader reader(datasetPath, cancellationToken);
        reader.addQueue(&frameQueue);
        reader.init();
        reader.runLoop();
    });

    Player player(frameQueue, cancellationToken);
    player.init();
    player.run();

    readerThread.join();

    return EXIT_SUCCESS;
}
