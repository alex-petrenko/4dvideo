#include <3dvideo/depth_filter.hpp>


DepthFilter::DepthFilter(FrameQueue &inputQueue, FrameProducer &output, CancellationToken &cancellationToken)
    : FrameConsumer(inputQueue, cancellationToken)
    , output(output)
{
}

void DepthFilter::process(std::shared_ptr<Frame> &frame)
{
    const uint16_t minDepth = 200, maxDepth = 1200, purgeRadius = 3;
    cv::Mat &depth = frame->depth;
    
    for (int i = 0; i < depth.rows; ++i)
        for (int j = 0; j < depth.cols; ++j)
        {
            uint16_t &d = depth.at<uint16_t>(i, j);
            if (d < minDepth || d > maxDepth)
            {
                d = 0;
                continue;
            }

            for (int di = 0; di <= purgeRadius && i + di < depth.rows; ++di)
                for (int dj = 0; dj <= purgeRadius && j + dj < depth.cols; ++dj)
                {
                    if (di == 0 && dj == 0) continue;
                    depth.at<uint16_t>(i + di, j + dj) = 0;
                }
        }

    output.produce(frame);
}
