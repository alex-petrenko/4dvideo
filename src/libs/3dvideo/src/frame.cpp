#include <3dvideo/frame.hpp>

#include <util/tiny_logger.hpp>


Frame::Frame(const cv::Mat &color, const cv::Mat &depth)
    : color(color)
    , depth(depth)
{
    TLOG(INFO) << color.cols << " " << color.rows << " depth: " << depth.cols << " " << depth.rows;
}

Frame::~Frame()
{
    TLOG(INFO);
}
