#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <functional>

#include <opencv2/core.hpp>

#include <util/macro.hpp>
#include <util/geometry.hpp>


/// Types.

typedef uint32_t EdgeIdx;

const int maxCoord = 1280;
const int maxNumPoints = 32767;
const uint32_t maxNumEdges = 512 * 1024;
const uint32_t maxNumTriangles = maxNumEdges / 2;
const EdgeIdx INVALID_EDGE = std::numeric_limits<EdgeIdx>::max();

// this structure is left unpacked, cause it turns out faster this way
struct TriEdge
{
    uint16_t origPnt;  // index of edge's origin point
    EdgeIdx symEdge;  // index of pair edge, with same endpoints and opposite direction
    EdgeIdx nextCcwEdge;  // next counterclockwise (CCW) edge around the origin
    EdgeIdx prevCcwEdge;  // previous CCW edge around the origin (or next CW edge)
};

class Delaunay
{
public:
    typedef void VisualizationCallback(const cv::Mat &);

private:
    class DelaunayImpl;

public:
    Delaunay();
    ~Delaunay();

    void operator()(std::vector<PointIJ> &points, std::vector<short> &indexMap);

    void init(const std::vector<PointIJ> &points);
    void sortPoints(std::vector<PointIJ> &points, std::vector<short> &indexMap);
    void triangulate();
    void generateTriangles();

    // visualization
    void plotTriangulation(cv::Mat &img);
    void showTriangulation();
    void setVisualizationCallback(const std::function<VisualizationCallback> &callback);

    static void saveTriangulation(const std::string &filename, int numP, const PointIJ *p, int numT, const Triangle *t);
    static void loadTriangulation(const std::string &filename, std::vector<PointIJ> &p, std::vector<Triangle> &t);

    bool isEqualTo(const std::vector<PointIJ> &p, const std::vector<Triangle> &t) const;

    void getTriangles(Triangle *&t, int &num);

private:
    std::unique_ptr<DelaunayImpl> data;
};

