#include <map>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <util/util.hpp>
#include <util/geometry.hpp>
#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>

#include <3dvideo/app_state.hpp>
#include <3dvideo/depth_filter.hpp>


using namespace std::chrono_literals;


DepthFilter::DepthFilter(FrameQueue &inputQueue, FrameProducer &output, CancellationToken &cancellationToken)
    : FrameConsumer(inputQueue, cancellationToken)
    , output(output)
{
}

void DepthFilter::init()
{
    TLOG(INFO);
    const SensorManager &sensorManager = appState().getSensorManager();
    while (!cancel && !sensorManager.isInitialized())
        std::this_thread::sleep_for(30ms);

    sensorManager.getColorParams(colorCam, Ignore<ColorDataFormat>());
    sensorManager.getDepthParams(depthCam, Ignore<DepthDataFormat>());
    calibration = sensorManager.getCalibration();
}

void DepthFilter::process(std::shared_ptr<Frame> &frame)
{
    tprof().startTimer("depth_filter");

    if (skipFiltering)
        return;

    const uint16_t minDepth = 500, maxDepth = 1100, purgeR = 3;
    const uint16_t curvatureThresholdMm = 12;
    const cv::Point3f *translation = reinterpret_cast<cv::Point3f *>(calibration.tvec.data);
    cv::Mat &depth = frame->depth;
    cv::Mat mask = cv::Mat::zeros(depth.rows, depth.cols, CV_8UC1);
    for (int i = 0; i < depth.rows; ++i)
        for (int j = 0; j < depth.cols; ++j)
        {
            uint16_t &d = depth.at<uint16_t>(i, j);
            if (d < minDepth || d > maxDepth || i < 1 || j < 1 || i >= depth.rows - 1 || j >= depth.cols - 1)
            {
                d = 0;
                continue;
            }

            const auto pointColorSpace = project2dPointTo3d(i, j, d, depthCam) + *translation;
            if (!project3dPointTo2d(pointColorSpace, colorCam, Ignore<int>(), Ignore<int>(), Ignore<uint16_t>()))
            {
                d = 0;
                continue;
            }

            bool masked = false;
            uint16_t maxDeltaMm = 0;
            for (int di = -1; di <= 1 && !masked; ++di)
                for (int dj = -1; dj <= 1; ++dj)
                {
                    const uint16_t deltaMm = uint16_t(std::abs(depth.at<uint16_t>(i + di, j + dj) - d));
                    maxDeltaMm = std::max(maxDeltaMm, deltaMm);
                    if (deltaMm > curvatureThresholdMm)
                    {
                        mask.at<uchar>(i, j) = 0xff;
                        masked = true;
                        break;
                    }
                }
        }

    int clusterIdx = 1;
    cv::Mat cluster = cv::Mat::zeros(depth.rows, depth.cols, CV_32SC1);
    std::queue<PointIJ> queue;
    std::map<int, int> clusterArea;
    for (short i = 0; i < depth.rows; ++i)
        for (short j = 0; j < depth.cols; ++j)
        {
            const uint16_t d = depth.at<uint16_t>(i, j);
            if (!d) continue;
            if (cluster.at<int>(i, j)) continue;

            cluster.at<int>(i, j) = clusterIdx;
            ++clusterArea[clusterIdx];
            queue.push({ i, j });

            while (!queue.empty())
            {
                const auto p = queue.front();
                queue.pop();

                for (short di = -1; di <= 1; ++di)
                    for (short dj = -1; dj <= 1; ++dj)
                    {
                        const short iNear = p.i + di, jNear = p.j + dj;
                        if (iNear < 0 || iNear >= depth.rows) continue;
                        if (jNear < 0 || jNear >= depth.cols) continue;
                        const uint16_t dNear = depth.at<uint16_t>(iNear, jNear);
                        if (!dNear) continue;
                        auto &c = cluster.at<int>(iNear, jNear);
                        if (c) continue;

                        c = clusterIdx;
                        ++clusterArea[clusterIdx];
                        queue.push({ iNear, jNear });
                    }
            }

            ++clusterIdx;
        }

    const int depthClusterAreaThreshold = int(depth.rows * depth.cols * 0.001);  // min 0.1% of the screen
    for (short i = 0; i < depth.rows; ++i)
        for (short j = 0; j < depth.cols; ++j)
        {
            uint16_t &d = depth.at<uint16_t>(i, j);
            if (!d) continue;
            const int c = cluster.at<int>(i, j);
            if (clusterArea[c] < depthClusterAreaThreshold)
                d = 0;
        }

    for (int i = 0; i < depth.rows; ++i)
        for (int j = 0; j < depth.cols; ++j)
        {
            if (!depth.at<uint16_t>(i, j)) continue;
            mask.at<uchar>(i, j) = 0xff;

            for (int iNear = std::max(i - purgeR, 0); iNear <= i + purgeR && iNear < depth.rows; ++iNear)
                for (int jNear = std::max(j - purgeR, 0); jNear <= j + purgeR && jNear < depth.cols; ++jNear)
                {
                    if (mask.at<uchar>(iNear, jNear)) continue;
                    depth.at<uint16_t>(iNear, jNear) = 0;
                }
        }

#define VISUALIZE_FILTER 0
#if VISUALIZE_FILTER
    cv::Mat filtered;
    depth.copyTo(filtered);
    for (int i = 0; i < filtered.rows; ++i)
        for (int j = 0; j < filtered.cols; ++j)
            if (filtered.at<uint16_t>(i, j) > 0)
                filtered.at<uint16_t>(i, j) = 16000;

    cv::resize(filtered, filtered, cv::Size(), 2, 2);
    cv::imshow("filtered", filtered);
    cv::waitKey();
#endif

    tprof().stopTimer("depth_filter");

    output.produce(frame);
}
