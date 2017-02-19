#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <util/util.hpp>
#include <util/geometry.hpp>


TEST(geom, inCircle)
{
    {
        const cv::Point2i a(0, 2), b(2, 0), c(-2, 0);
        inCircle(a.x, a.y, b.x, b.y, c.x, c.y, 0, 0);

        EXPECT_TRUE(inCircle(a.x, a.y, b.x, b.y, c.x, c.y, 0, 0));
        EXPECT_FALSE(inCircle(a.x, a.y, b.x, b.y, c.x, c.y, 0, 2));
        EXPECT_FALSE(inCircle(a.x, a.y, b.x, b.y, c.x, c.y, 100, 100));
    }

    {
        const cv::Point2i a(2, 2), b(1, 1), c(1, 3);
        EXPECT_FALSE(inCircle(2, 2, 1, 1, 1, 3, 1, 0));
        EXPECT_FALSE(inCircle(-6, -12, 0, -6, -6, 0, -12, -6));
    }

    {
        const int maxCenter = 6, maxCoord = 2 * maxCenter, maxR = 10;
        for (int r = 0; r <= maxR; ++r)
        {
            const double rSqr = sqr(r) - 1e-5;
            for (int cx = -maxCenter; cx <= maxCenter; ++cx)
                for (int cy = -maxCenter; cy <= maxCenter; ++cy)
                    for (int x = -maxCoord; x <= maxCoord; ++x)
                        for (int y = -maxCoord; y <= maxCoord; ++y)
                        {
                            const double distSqr = sqr(x - cx) + sqr(y - cy);
                            const bool in = inCircle(cx + r, cy, cx, cy - r, cx, cy + r, x, y);
                            if (distSqr > rSqr)
                                EXPECT_FALSE(in);
                            else
                                EXPECT_TRUE(in);
                        }
        }
    }
}

TEST(geom, triangleArea3D)
{
    const cv::Point3f a, b(1, 0, 0), c(0, 1, 0);
    float area = triangleArea3D(a, b, c);
    EXPECT_EQ(area, 0.5f);

    area = triangleArea3D(a, a, b);
    EXPECT_EQ(area, 0.0f);

    const cv::Point3f ab(b - a), bc(c - b), ca(a - c);
    const float na = float(cv::norm(ab)), nb = float(cv::norm(bc)), nc = float(cv::norm(ca));
    area = triangleArea3DHeron(na, nb, nc);
    EXPECT_NEAR(area, 0.5f, EPSILON);

    area = triangleArea3DHeronSq(na, nb, nc);
    EXPECT_NEAR(area, 0.25f, EPSILON);

    area = triangleArea3DHeron(0, 1, 1);
    EXPECT_NEAR(area, 0.0f, EPSILON);
}
