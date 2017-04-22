#include <vector>
#include <fstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <util/util.hpp>
#include <util/geometry.hpp>
#include <util/tiny_logger.hpp>

#include <3dvideo/frame.hpp>
#include <3dvideo/app_state.hpp>
#include <3dvideo/dataset_writer.hpp>


struct TangoPoint
{
    float x, y, z, c;
};

TangoPoint tangoPoints[100000];

float extrRotation[4];
float extrTranslation[3];


int main(int argc, char *argv[])
{
    const int numArgs = 3;
    if (argc != numArgs)
        TLOG(FATAL) << "Expected " << numArgs << " arguments, got " << argc;

    int arg = 1;
    const std::string datasetPath(argv[arg++]), outputPath(argv[arg++]);

    CancellationToken cancellationToken;
    FrameQueue frameQueue;

    std::thread writerThread([&]
    {
        DatasetWriter writer(R"(C:\temp\tst\converted.4dv)", frameQueue, cancellationToken);
        writer.init();
        writer.run();
    });

    auto &sensorManager = appState().getSensorManager();
    CameraParams colorCam;
    colorCam.w = 1280, colorCam.h = 720;
    sensorManager.setColorParams(colorCam, ColorDataFormat::YUV_NV21);

    CameraParams depthCam(520.9651f, 319.223f, 175.641f, 640, 360);
    sensorManager.setDepthParams(depthCam, DepthDataFormat::UNSIGNED_16BIT_MM);

    sensorManager.setInitialized();

    std::ifstream input(datasetPath, std::ios::binary);
    cv::Mat imageBgr(720, 1280, CV_8UC3);

    int numFrames = 0;
    while (input)
    {
        auto frame = std::make_shared<Frame>();

        double timestamp;
        input.read((char *)&timestamp, sizeof(timestamp));
        endianSwap(&timestamp);
        frame->cTimestamp = int64_t(timestamp);

        frame->color = cv::Mat(3 * colorCam.h / 2, colorCam.w, CV_8UC1);
        input.read((char *)frame->color.data, frame->color.total() * frame->color.elemSize());
        // cv::cvtColor(image, imageBgr, cv::COLOR_YUV2BGR_NV21);

        if (!input) break;
        double pointCloudTimestamp;
        input.read((char *)&pointCloudTimestamp, sizeof(pointCloudTimestamp));
        endianSwap(&pointCloudTimestamp);
        frame->dTimestamp = int64_t(pointCloudTimestamp);

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

            frame->depth = cv::Mat::zeros(depthCam.h, depthCam.w, CV_16UC1);
            int iImg, jImg;
            uint16_t depth;
            for (int i = 0; i < numPoints; ++i)
            {
                endianSwap(&tangoPoints[i].x);
                endianSwap(&tangoPoints[i].y);
                endianSwap(&tangoPoints[i].z);

                const auto p = cv::Point3f(tangoPoints[i].x, tangoPoints[i].y, tangoPoints[i].z);
                if (project3dPointTo2d(p, depthCam.f, depthCam.cx, depthCam.cy, depthCam.w, depthCam.h, iImg, jImg, depth))
                    frame->depth.at<uint16_t>(iImg, jImg) = 0xFFFF;//depth;
            }

            cv::imshow("test", frame->depth);
            cv::waitKey(300);

            frame->frameNumber = numFrames;
            frameQueue.put(frame);
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

    cancellationToken.trigger();
    writerThread.join();

    return EXIT_SUCCESS;
}
