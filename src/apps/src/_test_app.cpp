#include <chrono>
#include <thread>

#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/data_visualizer.hpp>


using namespace std::chrono_literals;

struct s
{
    cv::Mat m;
};
std::queue<std::shared_ptr<s>> q;
std::mutex mutex;

int main()
{
    std::thread th([&]
    {
        for (int i = 0; i < 1000; ++i)
        {
            std::this_thread::sleep_for(1ms);
            std::lock_guard<std::mutex> lock(mutex);
            auto p = std::make_shared<s>();
            p->m = cv::Mat::ones(100, 100, CV_8UC1);
            q.push(p);
            std::cout << "Produced " << i;
        }
    });

    double sum = 0;
    while (true)
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (q.empty()) continue;
        auto p = q.front();
        q.pop();
        lock.unlock();

        for (int i = 0; i < p->m.rows; ++i)
            for (int j = 0; j < p->m.cols; ++j)
                sum += p->m.at<uchar>(i, j);

        std::this_thread::sleep_for(2ms);
        std::cout << sum << std::endl;
    }

    th.join();

    return EXIT_SUCCESS;
}
