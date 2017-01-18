#include <gtest/gtest.h>

#include <util/io_3d.hpp>
#include <util/test_utils.hpp>
#include <util/filesystem_utils.hpp>


TEST(io3d, binaryPly)
{
    std::vector<cv::Point3f> cloud;
    std::vector<cv::Vec3b> colors;
    const std::string testCloudFilename{ pathJoin(getTestDataFolder(), "test_point_cloud.ply") };
    const bool isOk = loadBinaryPly(testCloudFilename, &cloud, &colors);

    EXPECT_TRUE(isOk);
    EXPECT_EQ(cloud.size(), 11973);
    EXPECT_EQ(colors.size(), 11973);
}

TEST(io3d, triangulation)
{
    //loadTriangulation()
}
