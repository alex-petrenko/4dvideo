#pragma once

#include <tri/triangulation.hpp>

#include <3dvideo/frame.hpp>


struct Triangle3D  // <-- temporary slow version, should replace with indexed mode
{
    cv::Point3f p1, p2, p3;

    cv::Point3f a() const { return p2 - p1; }
    cv::Point3f b() const { return p3 - p2; }
    cv::Point3f c() const { return p1 - p3; }
};

struct TriangleUV
{
    cv::Point2f p1, p2, p3;
};


struct MeshFrame
{
    std::shared_ptr<Frame> frame2D;

    std::vector<PointIJ> points;
    std::vector<cv::Point3f> cloud;
    std::vector<cv::Point2f> uv;

    std::vector<Triangle3D> triangles3D, trianglesNormals;
    std::vector<TriangleUV> trianglesUv;

    int num3DTriangles;
};

typedef ConcurrentQueue<std::shared_ptr<MeshFrame>> MeshFrameQueue;
typedef Producer<MeshFrameQueue> MeshFrameProducer;
typedef Consumer<MeshFrameQueue> MeshFrameConsumer;
