#include <gtest/gtest.h>

#include <opencv2/highgui.hpp>

#include <tri/triangulation.hpp>

#include <util/io_3d.hpp>
#include <util/geometry.hpp>
#include <util/test_utils.hpp>
#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>
#include <util/filesystem_utils.hpp>


class tri : public ::testing::Test
{
protected:
    Delaunay delaunay;
};


TEST_F(tri, delaunayEmpty)
{
    std::vector<PointIJ> empty;
    delaunay.init(empty);
    delaunay.triangulate();
}

TEST_F(tri, delaunayOneSegment)
{
    std::vector<PointIJ> points{ { 2,1 },{ 1,2 },{ 2,1 } };  // test duplicated point as well
    std::vector<short> indexMap(points.size());
    delaunay(points, indexMap);
}

TEST_F(tri, delaunayOneTriangle)
{
    std::vector<PointIJ> points{ { 0,0 },{ 2,2 },{ 1,3 } };
    std::vector<short> indexMap(points.size());
    delaunay(points, indexMap);
}

TEST_F(tri, delaunayTwoTriangles)
{
    std::vector<PointIJ> points{ { 6,0 },{ 4,4 },{ 1,5 },{ 0,1 } };
    std::vector<short> indexMap(points.size());
    delaunay(points, indexMap);
}

TEST_F(tri, delaunayBasic)
{
    // sample from this article: http://www.geom.uiuc.edu/~samuelp/del_project.html
    std::vector<PointIJ> points{ { 200,0 },{ 300,100 },{ 100,100 },{ 0,100 },{ 200,200 },{ 0,300 },{ 100,400 },{ 300,500 },{ 200,500 },{ 0,500 },{ 100,400 } };
    std::vector<short> indexMap(points.size());
    delaunay(points, indexMap);
}

TEST_F(tri, delaunayRandom)
{
    std::vector<PointIJ> points;
    int count = 100;
    for (int i = 0; i < 6 * count; ++i)
        points.push_back(PointIJ(rand() % 100 + 100, rand() % 100 + 100));
    for (int i = 0; i < 10 * count; ++i)
        points.push_back(PointIJ(rand() % 200 + 400, rand() % 230 + 400));
    for (int i = 0; i < 5 * count; ++i)
        points.push_back(PointIJ(rand() % 70 + 400, rand() % 70 + 300));
    for (int i = 0; i < 10 * count; ++i)
        points.push_back(PointIJ(rand() % 640, rand() % 640));

    std::vector<short> indexMap(points.size());
    delaunay(points, indexMap);
}

TEST_F(tri, delaunayCloud)
{
    std::vector<cv::Point3f> cloud;
    const std::string testCloudFilename{ pathJoin(getTestDataFolder(), "test_point_cloud.ply") };
    const bool isOk = loadBinaryPly(testCloudFilename, &cloud);
    EXPECT_TRUE(isOk);

    std::vector<PointIJ> points;
    const int w = 640, h = 360;
    const float f = 520.965f, cx = 319.223f, cy = 175.641f;  // parameters of Tango camera
    int iImg, jImg;
    ushort depth;
    for (size_t i = 0; i < cloud.size(); ++i)
        if (project3dPointTo2d(cloud[i], f, cx, cy, w, h, iImg, jImg, depth))
            points.emplace_back(iImg, jImg);

    TLOG(INFO) << "number of points: " << points.size();

    std::vector<PointIJ> pCopy;
    std::vector<short> indexMap(points.size());

#if defined(NDEBUG)
    const int numRuns = 1000;
#else
    const int numRuns = 10;
#endif

    for (int i = 0; i < numRuns; ++i)
    {
        indexMap.resize(points.size());
        pCopy = points;

        tprof().startTimer("tri");
        delaunay(pCopy, indexMap);
        tprof().pauseTimer("tri");

        tprof().startTimer("gentriangles");
        delaunay.generateTriangles();
        tprof().pauseTimer("gentriangles");
    }
    tprof().stopTimer("tri");
    tprof().stopTimer("gentriangles");

    cv::Mat triangImg;
    delaunay.plotTriangulation(triangImg);
    EXPECT_GT(triangImg.total(), 0);  // just in case, check if it draws something

    std::vector<PointIJ> p;
    std::vector<Triangle> t;
    delaunay.loadTriangulation(pathJoin(getTestDataFolder(), "reference.tri"), p, t);
    EXPECT_TRUE(delaunay.isEqualTo(p, t));  // protect from regression
}

TEST_F(tri, sorting)
{
    std::vector<PointIJ> points{ { 6,0 },{ 4,4 },{ 1,5 },{ 0,1 },{ 6,6 },{ 6,6 },{ 6,3 },{ 6,1 },{ 6,2 },{ 0,0 },{ 0,0 },{ 0,0 } };
    const std::vector<PointIJ> origPoints(points);
    std::vector<short> indexMap(points.size());
    delaunay.sortPoints(points, indexMap);

    EXPECT_LE(points.size(), origPoints.size());
    EXPECT_EQ(points.size(), indexMap.size());

    for (size_t i = 1; i < points.size(); ++i)
    {
        const PointIJ &prev = points[i - 1];
        const PointIJ &curr = points[i];

        EXPECT_LE(prev.j, curr.j);
        if (prev.j == curr.j)
            EXPECT_GT(prev.i, curr.i);  // sorted in reverse order by i-coordinate
    }

    for (size_t i = 0; i < indexMap.size(); ++i)
    {
        const short originalIdx = indexMap[i];
        EXPECT_EQ(origPoints[originalIdx], points[i]);
    }

    // verify that there are no duplicates left
    const auto endIt = std::unique(points.begin(), points.end());
    EXPECT_EQ(endIt, points.end());

    // check that we didn't throw away too many points
    const std::set<PointIJ> origPointSet(origPoints.begin(), origPoints.end());
    EXPECT_EQ(origPointSet.size(), points.size());
    for (const auto &p : origPointSet)
    {
        const auto it = std::find(points.begin(), points.end(), p);
        EXPECT_NE(it, points.end());  // all unique points should be in "points" vector
    }
}

TEST_F(tri, saveLoad)
{
    std::vector<PointIJ> p1, p2;
    std::vector<Triangle> t1, t2;
    delaunay.loadTriangulation(pathJoin(getTestDataFolder(), "reference.tri"), p1, t1);
    delaunay.saveTriangulation(pathJoin(getTestDataFolder(), "tmp_triangulation.tri"), int(p1.size()), p1.data(), int(t1.size()), t1.data());
    delaunay.loadTriangulation(pathJoin(getTestDataFolder(), "tmp_triangulation.tri"), p2, t2);

    EXPECT_FALSE(p1.empty());
    EXPECT_EQ(p1.size(), p2.size());
    EXPECT_EQ(memcmp(p1.data(), p2.data(), p1.size() * sizeof(p1.front())), 0);
    EXPECT_EQ(t1.size(), t2.size());
    EXPECT_EQ(memcmp(t1.data(), t2.data(), t1.size() * sizeof(t1.front())), 0);
}
