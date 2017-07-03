#include <gtest/gtest.h>

#include <util/test_utils.hpp>
#include <util/filesystem_utils.hpp>

#include <3dvideo/app_state.hpp>
#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/dataset_reader.hpp>


class binaryDataset : public ::testing::Test
{
public:
    ~binaryDataset()
    {
        appState().reset();  // dataset reader alters global state, better reset it after the test
    }
};


TEST_F(binaryDataset, readWrite)
{
    const std::string testDataset{ pathJoin(getTestDataFolder(), "test_binary_dataset.4dv") };
    const std::string testDatasetCopy{ pathJoin(getTestDataFolder(), "test_binary_dataset_verify.4dv") };

    CancellationToken cancellationToken;
    FrameQueue queue;

    appState().startGrabbing();

    std::thread writerThread([&]()
    {
        DatasetWriter writer(testDatasetCopy, queue, cancellationToken);
        writer.init();
        writer.run();
    });

    DatasetReader reader(testDataset, true, cancellationToken);
    reader.addQueue(&queue);
    reader.init();
    reader.run();

    cancellationToken.trigger();
    writerThread.join();

    std::ifstream originalFile(testDataset, std::ios::binary), copyFile(testDatasetCopy, std::ios::binary);
    std::vector<char> original, copy;
    const auto origBytes = readAllBytes(originalFile, original), copyBytes = readAllBytes(copyFile, copy);
    EXPECT_EQ(origBytes, copyBytes);
    EXPECT_TRUE(std::equal(original.begin(), original.end(), copy.begin()));
}
