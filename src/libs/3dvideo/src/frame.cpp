#include <3dvideo/frame.hpp>

#include <util/tiny_logger.hpp>


Frame::Frame(int idx, const cv::Mat &color, const cv::Mat &depth)
    : frameNumber(idx)
    , color(color)
    , depth(depth)
{
    TLOG(INFO) << color.cols << " " << color.rows << " depth: " << depth.cols << " " << depth.rows;
}

Frame::~Frame()
{
}
