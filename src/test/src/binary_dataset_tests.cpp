#include <gtest/gtest.h>

#include <util/test_utils.hpp>
#include <util/filesystem_utils.hpp>

#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/dataset_reader.hpp>


TEST(binaryDataset, readWrite)
{
    const std::string testDataset { pathJoin(getTestDataFolder(), "test_binary_dataset.4dv") };

    CancellationToken cancellationToken;
    FrameQueue queue;

    DatasetReader reader(testDataset, cancellationToken);
    reader.addQueue(&queue);
    reader.init();
    reader.run();
}
