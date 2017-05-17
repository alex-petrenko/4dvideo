#pragma once

#include <3dvideo/mesh_frame.hpp>


class AnimationWriter : public MeshFrameConsumer
{
public:
    AnimationWriter(const std::string &outputPath, MeshFrameQueue &q, CancellationToken &cancellationToken);
    virtual ~AnimationWriter();

protected:
    void process(std::shared_ptr<MeshFrame> &item) override;

private:
    std::string outputPath;
    std::ofstream timeframe;

    int lastWrittenFrame = -1;
};
