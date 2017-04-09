#include <vector>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <util/util.hpp>
#include <util/tiny_logger.hpp>


struct TangoPoint
{
    float x, y, z, c;
};

TangoPoint tangoPoints[100000];

struct Frame
{
    std::vector<cv::Point3f> frameCloud;
    double timestamp;
};
std::vector<Frame> frames;

float extrRotation[4];
float extrTranslation[3];


void writeHeader(const std::ofstream &output)
{
    TLOG(INFO) << "Writing file header";
}

int main(int argc, char *argv[])
{
    const int numArgs = 3;
    if (argc != numArgs)
        TLOG(FATAL) << "Expected " << numArgs << " arguments, got " << argc;

    int arg = 1;
    const std::string datasetPath(argv[arg++]), outputPath(argv[arg++]);

    std::ifstream input(datasetPath, std::ios::binary);
    std::ofstream output(outputPath, std::ios::binary);
    
    writeHeader(output);

    cv::Mat imageBgr(720, 1080, CV_8UC3);
    cv::Mat image(3 * 720 / 2, 1280, CV_8UC1);

    int numFrames = 0;
    while (input)
    {
        double timestamp;
        input.read((char *)&timestamp, sizeof(timestamp));
        endianSwap(&timestamp);
        input.read((char *)image.data, image.total());
        // cv::cvtColor(image, imageBgr, cv::COLOR_YUV2BGR_NV21);

        if (!input) break;
        double pointCloudTimestamp;
        input.read((char *)&pointCloudTimestamp, sizeof(pointCloudTimestamp));
        endianSwap(&pointCloudTimestamp);

        if (!input) break;
        int numPoints;
        input.read((char *)&numPoints, sizeof(numPoints));
        endianSwap(&numPoints);
        TLOG(INFO) << "point cloud timestamp: " << pointCloudTimestamp << " num points: " << numPoints << " size: " << numPoints * 4 * 4;

        if (numPoints > 0)
        {
            if (!input) break;
            int numPointBytes;
            input.read((char *)&numPointBytes, sizeof(numPointBytes));
            endianSwap(&numPointBytes);
            if (numPoints * 4 * 4 != numPointBytes)
                TLOG(INFO) << "Num point bytes: " << numPointBytes;
            if (!input) break;
            input.read((char *)tangoPoints, numPointBytes);

            frames.emplace_back();
            Frame &currentFrame = frames.back();
            currentFrame.timestamp = pointCloudTimestamp;
            currentFrame.frameCloud.resize(numPoints);
            for (int i = 0; i < numPoints; ++i)
            {
                endianSwap(&tangoPoints[i].x);
                endianSwap(&tangoPoints[i].y);
                endianSwap(&tangoPoints[i].z);
                currentFrame.frameCloud[i] = cv::Point3f(tangoPoints[i].x, tangoPoints[i].y, tangoPoints[i].z);
            }
        }

        if (!input) break;
        input.read((char *)extrRotation, sizeof(extrRotation));
        for (int i = 0; i < ARR_LENGTH(extrRotation); ++i)
            endianSwap(&extrRotation[i]);

        if (!input) break;
        input.read((char *)extrTranslation, sizeof(extrTranslation));
        for (int i = 0; i < ARR_LENGTH(extrTranslation); ++i)
            endianSwap(&extrTranslation[i]);

        ++numFrames;
        TLOG(INFO) << "num frames: " << numFrames;
    }

    return EXIT_SUCCESS;
}
