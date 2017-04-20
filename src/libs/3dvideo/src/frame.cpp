#include <3dvideo/frame.hpp>

#include <util/tiny_logger.hpp>


Frame::Frame()
{
}

Frame::Frame(int idx, const cv::Mat &color, int64_t cTimestamp, const cv::Mat &depth, int64_t dTimestamp)
    : frameNumber(idx)
    , color(color)
    , cTimestamp(cTimestamp)
    , depth(depth)
    , dTimestamp(dTimestamp)
{
    TLOG(INFO) << color.cols << " " << color.rows << " depth: " << depth.cols << " " << depth.rows;
}

Frame::~Frame()
{
}
