#pragma once

#include <tri/triangulation.hpp>

#include <3dvideo/mesh_frame.hpp>


class Mesher : public FrameConsumer
{
public:
    class MesherImpl;

public:
    Mesher(FrameQueue &inputQueue, MeshFrameProducer &output, CancellationToken &cancellationToken);

    void init() override;

protected:
    void process(std::shared_ptr<Frame> &frame) override;

private:
    void fillPoints(const cv::Mat &depth, std::vector<PointIJ> &points, std::vector<cv::Point3f> &cloud) const;
    void fillPoints(const std::vector<cv::Point3f> &frameCloud, std::vector<PointIJ> &points, std::vector<cv::Point3f> &cloud) const;

private:
    MeshFrameProducer &output;

    Delaunay delaunay;

    CameraParams colorCam, depthCam;
    Calibration calibration;
    float scale;
};
