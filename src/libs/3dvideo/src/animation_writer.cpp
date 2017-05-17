#include <iomanip>

#include <util/io_3d.hpp>
#include <util/filesystem_utils.hpp>

#include <3dvideo/animation_writer.hpp>


AnimationWriter::AnimationWriter(const std::string &outputPath, MeshFrameQueue &q, CancellationToken &cancellationToken)
    : MeshFrameConsumer(q, cancellationToken)
    , outputPath(outputPath)
    , timeframe(pathJoin(outputPath, "sketchfab.timeframe"), std::ios::out)
{
}

AnimationWriter::~AnimationWriter()
{
}

void AnimationWriter::process(std::shared_ptr<MeshFrame> &frame)
{
    if (frame->frame2D->frameNumber < lastWrittenFrame)
    {
        timeframe.close();
        return;
    }

    std::ostringstream fileName;
    fileName << std::setw(5) << std::setfill('0') << frame->frame2D->frameNumber << "_cloud.ply";

    const std::string filePath = pathJoin(outputPath, fileName.str());
    saveBinaryPly(filePath, &frame->cloud);

    timeframe << "0.05 " << fileName.str() << '\n';
    lastWrittenFrame = frame->frame2D->frameNumber;
}
