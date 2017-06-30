#include <memory>
#include <thread>
#include <iomanip>
#include <sstream>

#include <opencv2/highgui.hpp>

#include <util/util.hpp>
#include <util/consumer.hpp>
#include <util/producer.hpp>
#include <util/tiny_logger.hpp>
#include <util/concurrent_queue.hpp>

#include <tri/triangulation.hpp>


typedef ConcurrentQueue<std::shared_ptr<cv::Mat>> VisQueue;

namespace
{

void scalePoints(std::vector<PointIJ> &v, float factor)
{
    for (auto &p : v)
        p.i = short(p.i * factor), p.j = short(p.j * factor);
}

void alignPoints(std::vector<PointIJ> &v)
{
    short minI = v.front().i, minJ = v.front().j;
    for (const auto &p : v)
        minI = std::min(minI, p.i), minJ = std::min(minJ, p.j);
    for (auto &p : v)
        p.i -= minI, p.j -= minJ;
}

std::vector<PointIJ> getPointsSample()
{
    // sample from this article: http://www.geom.uiuc.edu/~samuelp/del_project.html
    const std::vector<PointIJ> points{ { 200,0 },{ 300,100 },{ 100,100 },{ 0,100 },{ 200,200 },{ 0,300 },{ 100,400 },{ 300,500 },{ 200,500 },{ 0,500 },{ 100,400 } };
    return points;
}

std::vector<PointIJ> getPointsSampleDoubled()
{
    auto points = getPointsSample();
    points.resize(points.size() * 2);
    for (int i = 0; i < points.size() / 2; ++i)
    {
        auto &p = points[i + points.size() / 2];
        p.i = 500 - points[i].i;
        p.j = points[i].j;
    }
    return points;
}

std::vector<PointIJ> getPointsGrid(int n = 9, int step = 31)
{
    std::vector<PointIJ> p;
    for (int i = 1; i < n; ++i)
        for (int j = 1; j < n; ++j)
            p.emplace_back(i * step, j * step);
    return p;
}

std::vector<PointIJ> getPointsCircle(int n = 60, int r = 160, int cx = 160, int cy = 160)
{
    std::vector<PointIJ> p;
    const double step = 2 * M_PI / n;
    double t = 0;
    while (t < 2 * M_PI)
    {
        const int x = int(cos(t) * r + cx + 0.5), y = int(sin(t) * r + cy + 0.5);
        p.emplace_back(y, x);
        t += step;
    }
    return p;
}

std::vector<PointIJ> getPointsCircleCenter(int n = 60, int r = 160, int cx = 160, int cy = 160)
{
    auto p = getPointsCircle(n, r, cx, cy);
    p.emplace_back(cy, cx);
    return p;
}

std::vector<PointIJ> getPointsCirclesGrid(bool withCenters = false)
{
    const auto grid = getPointsGrid(6, 150);
    std::vector<PointIJ> points, circle;

    int nMin = 3, nMax = 20, rMax = 65;
    for (const auto &p : grid)
    {
        const int n = randRange(nMin, nMax);
        const int rMin = int(35.0f * (float(n) / nMax));
        const int r = randRange(rMin, rMax);

        if (withCenters)
            circle = getPointsCircleCenter(n, r, p.j, p.i);
        else
            circle = getPointsCircle(n, r, p.j, p.i);
        points.insert(points.end(), circle.begin(), circle.end());
    }

    return points;
}

std::vector<PointIJ> getPointsCirclesCircle(bool withCenters = false)
{
    constexpr int N = 12, R = 180, n = 24, r = 42;
    std::vector<PointIJ> points, circle, outerCircle;
    if (withCenters)
        outerCircle = getPointsCircleCenter(N, R);
    else
        outerCircle = getPointsCircle(N, R);

    for (const auto &p : outerCircle)
    {
        if (withCenters)
            circle = getPointsCircleCenter(n, r, p.j, p.i);
        else
            circle = getPointsCircle(n, r, p.j, p.i);
        points.insert(points.end(), circle.begin(), circle.end());
    }

    return points;
}

std::vector<PointIJ> getPointsRandom()
{
    constexpr int n = 500;
    std::vector<PointIJ> p;
    for (int i = 0; i < n; ++i)
    {
        const short x = rand() % (1920 / 2), y = rand() % (1080 / 2);
        p.emplace_back(y, x);
    }
    return p;
}

std::vector<PointIJ> getPointsDataset()
{
    const std::string imgPath = R"(C:\temp\tst\anim\00000001_frame.bmp)";
    const auto img = cv::imread(imgPath, cv::IMREAD_UNCHANGED);
    std::vector<PointIJ> p;
    const cv::Vec3b zero{ 0, 0, 0 };
    for (int i = 0; i < img.rows; ++i)
        for (int j = 0; j < img.cols; ++j)
            if (img.at<cv::Vec3b>(i, j) != zero)
                p.emplace_back(i, j);
    return p;
}

void showOnScreen(const cv::Mat &img, int delayMs = 1)
{
    cv::imshow("vis", img);
    cv::waitKey(delayMs);
}

void saveToDisk(const cv::Mat &img)
{
    static int numFrames = 0;
    std::stringstream s;
    s << R"(C:\all\projects\personal\3dvideo_data\article\animations\14_dataset\)";
    s << "frame_" << std::setw(8) << std::setfill('0') << numFrames << ".png";
    ++numFrames;
    const auto path = s.str();
    cv::imwrite(path, img);
}

class VisProcessor : public Consumer<VisQueue>
{
public:
    VisProcessor(VisQueue &q, CancellationToken &cancel)
        : Consumer(q, cancel)
    {
    }

    virtual ~VisProcessor() override
    {
        if (lastFrame)
            pauseAtTheEnd();
    }

protected:
    virtual void process(std::shared_ptr<cv::Mat> &frame) override
    {
        saveToDisk(*frame);
        lastFrame = frame;
    }

private:
    void pauseAtTheEnd(int fps = 120, int delaySeconds = 4)
    {
        for (int i = 0; i < fps * delaySeconds; ++i)
            saveToDisk(*lastFrame);
    }

private:
    std::shared_ptr<cv::Mat> lastFrame;
};

}

int main()
{
    using namespace std::placeholders;

    auto points = getPointsDataset();
    alignPoints(points);
    std::vector<short> indexMap(points.size());

    CancellationToken cancellationToken;
    VisQueue q{ 2000 };
    Producer<VisQueue> visProducer(cancellationToken);
    visProducer.addQueue(&q);

    std::thread processingThread([&]
    {
        VisProcessor visProcessor(q, cancellationToken);
        visProcessor.init();
        visProcessor.run();
    });

    Delaunay delaunay;
    constexpr bool writeToDisk = true;
    if (writeToDisk)
    {
        delaunay.setVisualizationCallback([&](const cv::Mat &img)
        {
            auto copy = std::make_shared<cv::Mat>();
            img.copyTo(*copy);
            visProducer.produce(copy);
        });
    }
    else
        delaunay.setVisualizationCallback(std::bind(showOnScreen, _1, 10));

    delaunay(points, indexMap);
    if (!writeToDisk)
        cv::waitKey();

    cancellationToken.trigger();
    processingThread.join();

    return EXIT_SUCCESS;
}
