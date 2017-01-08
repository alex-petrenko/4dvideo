#pragma once

#include <vector>
#include <fstream>
#include <cstdint>

#include <opencv2/core.hpp>

#include <util/macro.hpp>


/// Types.

// represents pixel on the image
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


typedef uint32_t EdgeIdx;

const int maxCoord = 640;
const int maxNumPoints = 32767;
const uint32_t maxNumEdges = 512 * 1024;
const uint32_t maxNumTriangles = maxNumEdges / 2;
const EdgeIdx INVALID_EDGE = std::numeric_limits<EdgeIdx>::max();

void sortPoints(std::vector<PointIJ> &points, std::vector<short> &indexMap);
void triangulate(std::vector<PointIJ> &points, EdgeIdx &le);
void showTriangulation(cv::Mat &img, EdgeIdx le, EdgeIdx re = INVALID_EDGE, EdgeIdx base = INVALID_EDGE);
void generateTriangles(EdgeIdx startEdge);
