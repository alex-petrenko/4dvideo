#include <gtest/gtest.h>

#include <tri/triangulation.hpp>


TEST(tri, delaunayEmpty)
{
    std::vector<PointIJ> empty;
    EdgeIdx edgeIdx;
    triangulate(empty, edgeIdx);
}

TEST(tri, delaunayOneSegment)
{
    std::vector<PointIJ> points{ { 2,1 },{ 1,2 },{ 2,1 } };  // test duplicated point as well
    std::vector<short> indexMap(points.size());
    sortPoints(points, indexMap);
    EdgeIdx edgeIdx;
    triangulate(points, edgeIdx);
}

TEST(tri, delaunayOneTriangle)
{
    std::vector<PointIJ> points{ { 0,0 },{ 2,2 },{ 1,3 } };
    std::vector<short> indexMap(points.size());
    sortPoints(points, indexMap);
    EdgeIdx edgeIdx;
    triangulate(points, edgeIdx);
}

TEST(tri, delaunayTwoTriangles)
{
    std::vector<PointIJ> points{ { 6,0 },{ 4,4 },{ 1,5 },{ 0,1 } };
    std::vector<short> indexMap(points.size());
    sortPoints(points, indexMap);
    EdgeIdx edgeIdx;
    triangulate(points, edgeIdx);
}

TEST(tri, delaunayBasic)
{
    // sample from this article: http://www.geom.uiuc.edu/~samuelp/del_project.html
    std::vector<PointIJ> points{ { 200,0 },{ 300,100 },{ 100,100 },{ 0,100 },{ 200,200 },{ 0,300 },{ 100,400 },{ 300,500 },{ 200,500 },{ 0,500 },{ 100,400 } };
    std::vector<short> indexMap(points.size());
    sortPoints(points, indexMap);
    EdgeIdx edgeIdx;
    triangulate(points, edgeIdx);
}

TEST(tri, delaunayRandom)
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

    EdgeIdx edgeIdx;
    std::vector<short> indexMap(points.size());
    sortPoints(points, indexMap);
    triangulate(points, edgeIdx);
}

TEST(tri, sorting)
{
    std::vector<PointIJ> points{ { 6,0 },{ 4,4 },{ 1,5 },{ 0,1 },{ 6,6 },{ 6,6 },{ 6,3 },{ 6,1 },{ 6,2 },{ 0,0 },{ 0,0 },{ 0,0 } };
    const std::vector<PointIJ> origPoints(points);
    std::vector<short> indexMap(points.size());
    sortPoints(points, indexMap);

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
