#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

#include <util/macro.hpp>


/// Types.

/// Represents pixel on the image.
#pragma pack(push, 1)
struct PointIJ
{
    PointIJ()
        : i(0)
        , j(0)
    {
    }

    PointIJ(short i, short j)
        : i(i)
        , j(j)
    {
    }

    // Note! Sorts by x-coordinate first (j first)
    FORCE_INLINE bool operator<(const PointIJ &p) const
    {
        if (j != p.j)
            return j < p.j;
        else
            return p.i < i;
    }

    FORCE_INLINE bool operator==(const PointIJ &p) const
    {
        return i == p.i && j == p.j;
    }

    FORCE_INLINE bool operator!=(const PointIJ &p) const
    {
        return i != p.i || j != p.j;
    }

    friend std::ostream & operator<<(std::ostream &stream, const PointIJ &p)
    {
        stream << "(" << p.i << "," << p.j << ")";
        return stream;
    }

    short i, j;
};
#pragma pack(pop)

/// Triangle described by three vertex indices.
struct Triangle
{
    uint16_t p1, p2, p3;
};

enum Orientation
{
    ORIENT_CW,
    ORIENT_CCW,
    ORIENT_COLLINEAR,
};


/// Functions.

inline bool project3dPointTo2d(const cv::Point3f &p,
                               double f,
                               double cx,
                               double cy,
                               int w,
                               int h,
                               int &iImg,
                               int &jImg,
                               uint16_t &depth)
{
    const float fDivZ = float(f) / p.z;
    iImg = int(fDivZ * p.y + cy);
    if (iImg < 0 || iImg >= h)
        return false;
    jImg = int(fDivZ * p.x + cx);
    if (jImg < 0 || jImg >= w)
        return false;
    depth = uint16_t(p.z * 1000);  // depth is in millimeters
    return true;
}

FORCE_INLINE Orientation triOrientation(int x1, int y1, int x2, int y2, int x3, int y3)
{
    const int val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
    if (val < 0)
        return ORIENT_CW;
    else if (val > 0)
        return ORIENT_CCW;
    else
        return ORIENT_COLLINEAR;
}

/// Return true if point (px, py) lies strictly inside circle passing through points 1,2,3.
/// IMPORTANT! Points 1,2,3 should be oriented counterclockwise!
FORCE_INLINE bool inCircle(int x1, int y1, int x2, int y2, int x3, int y3, int px, int py)
{
    // reduce the computational complexity by substracting the last row of the matrix
    // ref: https://www.cs.cmu.edu/~quake/robust.html
    const int p1p_x = x1 - px;
    const int p1p_y = y1 - py;

    const int p2p_x = x2 - px;
    const int p2p_y = y2 - py;

    const int p3p_x = x3 - px;
    const int p3p_y = y3 - py;

    const int64_t p1p = p1p_x * p1p_x + p1p_y * p1p_y;
    const int64_t p2p = p2p_x * p2p_x + p2p_y * p2p_y;
    const int64_t p3p = p3p_x * p3p_x + p3p_y * p3p_y;

    // determinant of matrix, see paper for the reference
    const int64_t res = p1p_x * (p2p_y * p3p - p2p * p3p_y)
                      - p1p_y * (p2p_x * p3p - p2p * p3p_x)
                      + p1p * (p2p_x * p3p_y - p2p_y * p3p_x);

    assert(std::abs(res) < std::numeric_limits<int64>::max() / 100);

    return res < 0;
}