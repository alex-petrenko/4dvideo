#include <iomanip>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <util/io_3d.hpp>
#include <util/tiny_logger.hpp>
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
    TLOG(INFO);

    timeframe << totalDelta / numFrames << " " << lastMeshFilename << '\n';
    timeframe.close();
}

void AnimationWriter::process(std::shared_ptr<MeshFrame> &frame)
{
    if (finished)
        return;

    if (!frame->indexedMode || frame->frame2D->frameNumber < lastWrittenFrame)
    {
        timeframe.close();
        finished = true;
        return;
    }

    if (!meanPointCalculated)
        modelCenter = meanPoint(frame->cloud), meanPointCalculated = true;

    TLOG(INFO) << "timeframe animation, frame #" << frame->frame2D->frameNumber;

    std::ostringstream filenamePrefix;
    filenamePrefix << std::setw(4) << std::setfill('0') << frame->frame2D->frameNumber;
    const std::string meshFilename = filenamePrefix.str() + ".ply";

    std::vector<cv::Point3f> points(frame->cloud);
    for (size_t i = 0; i < points.size(); ++i)
    {
        points[i] -= modelCenter;
        points[i].x *= -1, points[i].y *= -1;
    }

    const bool withColor = !frame->frame2D->color.empty();
    if (withColor)
    {
        const std::string textureFilename = filenamePrefix.str() + ".jpg";

        std::vector<cv::Point2f> uv(frame->uv.size());
        for (size_t i = 0; i < uv.size(); ++i)
            uv[i].x = frame->uv[i].x, uv[i].y = 1.0f - frame->uv[i].y;
        saveBinaryPly(pathJoin(outputPath, meshFilename), &points, &frame->triangles, &uv, &textureFilename);

        cv::Mat finalTexture;
        cv::resize(frame->frame2D->color, finalTexture, cv::Size(), 0.25, 0.25, CV_INTER_CUBIC);
        cv::imwrite(pathJoin(outputPath, textureFilename), finalTexture);
    }
    else
        saveBinaryPly(pathJoin(outputPath, meshFilename), &points, &frame->triangles);

    if (lastWrittenFrame != -1)
    {
        const auto timeDeltaSeconds = float(frame->frame2D->dTimestamp - lastFrameTimestamp) / 1000000;
        timeframe << std::setprecision(3) << timeDeltaSeconds << " " << lastMeshFilename << '\n';
        totalDelta += timeDeltaSeconds;
    }

    lastFrameTimestamp = frame->frame2D->dTimestamp;
    lastWrittenFrame = frame->frame2D->frameNumber;
    lastMeshFilename = meshFilename;
    ++numFrames;
}
