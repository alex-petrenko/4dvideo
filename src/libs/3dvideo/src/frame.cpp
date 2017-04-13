#include <3dvideo/frame.hpp>

#include <util/tiny_logger.hpp>


Frame::Frame(const cv::Mat &color)
    : color(color)
{
    TLOG(INFO) << color.cols << " " << color.rows;
}

Frame::~Frame()
{
    TLOG(INFO);
}
