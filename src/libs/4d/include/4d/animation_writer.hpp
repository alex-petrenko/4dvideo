#pragma once

#include <4d/mesh_frame.hpp>


class AnimationWriter : public MeshFrameConsumer
{
public:
    AnimationWriter(const std::string &outputPath, MeshFrameQueue &q, CancellationToken &cancellationToken);
    virtual ~AnimationWriter();

protected:
    void process(std::shared_ptr<MeshFrame> &item) override;

private:
    bool finished = false;

    std::string outputPath;
    std::ofstream timeframe;

    cv::Point3f modelCenter;
    bool meanPointCalculated = false;

    int lastWrittenFrame = -1;
    int64_t lastFrameTimestamp = -1;
    std::string lastMeshFilename;

    float totalDelta = 0;
    int numFrames = 0;
};
