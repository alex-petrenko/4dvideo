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
    return;

    if (frame->frame2D->dTimestamp == firstFrameTimestamp)
    {
        timeframe.close();
        return;
    }

    if (lastWrittenFrame == -1)
        firstFrameTimestamp = frame->frame2D->dTimestamp;

    std::ostringstream fileName;
    fileName << std::setw(5) << std::setfill('0') << frame->frame2D->frameNumber << "_cloud.ply";

    const std::string filePath = pathJoin(outputPath, fileName.str());
    saveBinaryPly(filePath, &frame->cloud);

    auto timeDeltaUs = float(frame->frame2D->dTimestamp - firstFrameTimestamp) / 1000000;
    timeDeltaUs = std::max(timeDeltaUs, 0.03f);

    timeframe << timeDeltaUs << " " << fileName.str() << '\n';
    lastWrittenFrame = frame->frame2D->frameNumber;
}
