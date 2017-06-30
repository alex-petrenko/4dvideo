#pragma once

#include <tri/triangulation.hpp>

#include <3dvideo/mesh_frame.hpp>


class Mesher : public FrameConsumer
{
private:
    class MesherImpl;

public:
    Mesher(FrameQueue &inputQueue, MeshFrameProducer &output, CancellationToken &cancellationToken);

    void init() override;

protected:
    void process(std::shared_ptr<Frame> &frame) override;

private:
    void fillPoints(const cv::Mat &depth, std::vector<PointIJ> &points, std::vector<cv::Point3f> &cloud) const;
    void fillPoints(const std::vector<cv::Point3f> &frameCloud, std::vector<PointIJ> &points, std::vector<cv::Point3f> &cloud) const;

    void fillDataArrayMode(MeshFrame &frame, Triangle *triangles, int numTriangles);
    void fillDataIndexedMode(MeshFrame &frame, Triangle *triangles, int numTriangles);

private:
    constexpr static bool skipFiltering = false;

    MeshFrameProducer &output;

    Delaunay delaunay;

    CameraParams colorCam, depthCam;
    Calibration calibration;
    float scale;
};
