#include <chrono>
#include <thread>

#include <opencv2/highgui.hpp>

#include <util/io_3d.hpp>
#include <util/tiny_logger.hpp>

#include <3dvideo/dataset_reader.hpp>
#include <3dvideo/dataset_writer.hpp>
#include <3dvideo/data_visualizer.hpp>


int main()
{
    std::vector<cv::Point3f> v;
    loadBinaryPly(R"(C:\temp\tst\anim\00003_mesh.ply)", &v);

    const int w = 640, h = 360;
    cv::Mat img = cv::Mat::zeros(h, w, CV_8UC3);
    const float f = 520.965f, cx = 319.223f, cy = 175.641f;  // parameters of Tango camera
    int iImg, jImg;
    ushort depth;

    for (size_t i = 0; i < v.size(); ++i)
    {
        v[i] *= 1000;
        if (project3dPointTo2d(v[i], f, cx, cy, w, h, iImg, jImg, depth))
        {
            auto &pixel = img.at<cv::Vec3b>(iImg, jImg);
            pixel[0] = 0xff;
        }
    }

    cv::imshow("proj", img);
    cv::waitKey();

    return 0;
}
