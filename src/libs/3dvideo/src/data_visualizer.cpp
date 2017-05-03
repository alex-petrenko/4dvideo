#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <util/geometry.hpp>
#include <util/tiny_logger.hpp>

#include <3dvideo/app_state.hpp>
#include <3dvideo/data_visualizer.hpp>


using namespace std::chrono_literals;


namespace
{

constexpr char escape = 27;

// helper functions

inline void resizeImg(const cv::Mat &img, cv::Mat &dst, int w, int h)
{
    if (img.cols != w || img.rows != h)
        cv::resize(img, dst, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
    else
        dst = img;
}

void handleEvents(CancellationToken &cancel)
{
    const auto key = cv::waitKey(15);
    if (key == ' ')
    {
        if (!appState().isGrabbingStarted())
            appState().startGrabbing();
        else
            appState().stopGrabbing();
    }
    else if (key == escape)
    {
        TLOG(INFO) << "Exiting...";
        appState().stopGrabbing();
        cancel.trigger();
    }
}

}


DataVisualizer::DataVisualizer(FrameQueue &q, CancellationToken &cancellationToken)
    : FrameConsumer(q, cancellationToken)
{
    TLOG(INFO);
    cv::namedWindow(windowName);
}

DataVisualizer::~DataVisualizer()
{
    TLOG(INFO);
    cv::destroyAllWindows();
}

void DataVisualizer::init()
{
    const SensorManager &sensorManager = appState().getSensorManager();
    while (!cancel && !sensorManager.isInitialized())
        std::this_thread::sleep_for(30ms);

    ColorDataFormat cFormat;
    DepthDataFormat dFormat;
    sensorManager.getColorParams(colorCamera, cFormat), sensorManager.getDepthParams(depthCamera, dFormat);
}

/// Visualizer uses OpenCV UI and thus requires this overload to be able to handle GUI events.
void DataVisualizer::run()
{
    while (!cancel)
    {
        handleEvents(cancel);
        loopBody();
    }
}

void DataVisualizer::process(std::shared_ptr<Frame> &frame)
{
    const int w = std::min(colorCamera.w, depthCamera.w), h = std::min(colorCamera.h, depthCamera.h);
    cv::Mat color, depth, colorWithDepth = cv::Mat::zeros(h, w, CV_8UC3);
    resizeImg(frame->color, color, w, h);

    if (!frame->depth.empty())
        resizeImg(frame->depth, depth, w, h);
    else
    {
        cv::Mat projection = cv::Mat::zeros(depthCamera.h, depthCamera.w, CV_16UC1);
        int iImg, jImg;
        uint16_t d;
        for (const auto &p : frame->cloud)
            if (project3dPointTo2d(p, depthCamera, iImg, jImg, d))
                projection.at<uint16_t>(iImg, jImg) = d;

        resizeImg(projection, depth, w, h);
    }

    const float maxColor = 40, maxDistMm = 6000, minDistMm = 300;

    for (int i = 0; i < depth.rows; ++i)
    {
        const ushort * const row = depth.ptr<ushort>(i);
        for (int j = 0; j < depth.cols; ++j)
        {
            const ushort d = row[j];
            const auto c = color.at<cv::Vec3b>(i, j);
            auto &res = colorWithDepth.at<cv::Vec3b>(i, j);
            res = c;

            if (d <= minDistMm || d >= maxDistMm)
                continue;

            const int depthColor = int(maxColor - d * maxColor / maxDistMm);
            res[1] = uchar(std::min(c[1] + depthColor, 255));
        }
    }

    cv::imshow(windowName, colorWithDepth);
}
