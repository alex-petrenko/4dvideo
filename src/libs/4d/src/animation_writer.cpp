#include <iomanip>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <util/io_3d.hpp>
#include <util/tiny_logger.hpp>
#include <util/filesystem_utils.hpp>

#include <4d/params.hpp>
#include <4d/animation_writer.hpp>


namespace
{

std::string frameFilename(int frameNumber, const std::string &ext)
{
    std::ostringstream filenamePrefix;
    filenamePrefix << std::setw(4) << std::setfill('0') << frameNumber;
    return filenamePrefix.str() + ext;
}

}


AnimationWriter::AnimationWriter(const std::string &outputPath, MeshFrameQueue &q, CancellationToken &cancellationToken)
    : MeshFrameConsumer(q, cancellationToken)
    , outputPath(outputPath)
    , timeframe(pathJoin(outputPath, "sketchfab.timeframe"), std::ios::out)
{
}

AnimationWriter::~AnimationWriter()
{
    TLOG(INFO);

    processFrameBatch();

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

    batch.emplace_back(frame);
    if (batch.size() >= animationParams().batchSize)
        processFrameBatch();

}

void AnimationWriter::processFrameBatch()
{
    if (batch.empty())
        return;

    const auto &firstFrame = batch.front()->frame2D;
    TLOG(INFO) << "Processing batch starting from frame #" << firstFrame->frameNumber;

    const bool withColor = !firstFrame->color.empty();

    const float textureScale = animationParams().textureScale;
    const cv::Mat &firstColor = firstFrame->color;
    const int texRows = int(firstColor.rows * textureScale), texCols = int(firstColor.cols * textureScale);
    cv::Mat atlas = cv::Mat::zeros(int(texRows * batch.size()), texCols, firstColor.type());
    const auto atlasName = frameFilename(firstFrame->frameNumber, ".jpg");

    const float uvStep = 1.0f / batch.size();
    for (size_t batchI = 0; batchI < batch.size(); ++batchI)
    {
        auto &frame = batch[batchI];

        const auto meshFilename = frameFilename(frame->frame2D->frameNumber, ".ply");

        std::vector<cv::Point3f> points(frame->cloud);
        for (size_t i = 0; i < points.size(); ++i)
        {
            points[i] -= modelCenter;
            points[i].x *= -1, points[i].y *= -1;
        }

        if (withColor)
        {
            std::vector<cv::Point2f> uv(frame->uv.size());
            for (size_t i = 0; i < uv.size(); ++i)
            {
                uv[i].x = frame->uv[i].x, uv[i].y = 1.0f - frame->uv[i].y;  // convert to .ply convention
                uv[i].y = (batch.size() - batchI - 1) * uvStep + uv[i].y / batch.size();  // convert to atlas uv-coordinates
            }
            saveBinaryPly(pathJoin(outputPath, meshFilename), &points, &frame->triangles, &uv, &atlasName);

            cv::Mat resizedTexture;
            cv::resize(frame->frame2D->color, resizedTexture, cv::Size(), textureScale, textureScale, CV_INTER_CUBIC);
            assert(resizedTexture.rows == texRows && resizedTexture.cols == texCols);
            resizedTexture.copyTo(atlas.rowRange(int(batchI * texRows), int((batchI + 1) * texRows)));
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

    batch.clear();

    if (withColor)
        cv::imwrite(pathJoin(outputPath, atlasName), atlas);
}
