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