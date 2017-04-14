#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <3dvideo/grabber_visualizer.hpp>


namespace
{

// helper functions

inline void resizeImg(const cv::Mat &img, cv::Mat &dst, int w, int h)
{
    if (img.cols != w || img.rows != h)
        cv::resize(img, dst, cv::Size(w, h), 0, 0, cv::INTER_NEAREST);
    else
        dst = img;
}

}


GrabberVisualizer::GrabberVisualizer(FrameQueue &q)
    : FrameConsumer(q)
{
    TLOG(INFO);
}

GrabberVisualizer::~GrabberVisualizer()
{
    TLOG(INFO);
}

void GrabberVisualizer::process(std::shared_ptr<Frame> &frame)
{
    TLOG(INFO);

    const int w = std::min(frame->color.cols, frame->depth.cols), h = std::min(frame->color.rows, frame->depth.rows);
    cv::Mat color, depth, colorWithDepth(h, w, CV_8UC3);
    resizeImg(frame->color, color, w, h);
    resizeImg(frame->depth, depth, w, h);

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
    cv::waitKey(15);
}
