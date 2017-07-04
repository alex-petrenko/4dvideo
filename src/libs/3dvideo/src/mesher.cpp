#include <thread>

#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>

#include <3dvideo/mesher.hpp>
#include <3dvideo/params.hpp>
#include <3dvideo/player.hpp>
#include <3dvideo/app_state.hpp>


using namespace std::chrono_literals;


namespace
{

/// Filter triangles with too much projection area.
bool filterTriangle2D(const PointIJ &a, const PointIJ &b, const PointIJ &c)
{
    const cv::Point2f p1(a.j, a.i), p2(b.j, b.i), p3(c.j, c.i);
    const double threshold = mesherParams().sideLengthThreshold2D;
    if (cv::norm(p2 - p1) > threshold) return true;
    if (cv::norm(p3 - p2) > threshold) return true;
    if (cv::norm(p1 - p3) > threshold) return true;
    return false;
}

bool filterTriangle3D(const cv::Point3f &p1, const cv::Point3f &p2, const cv::Point3f &p3)
{
    const float zThreshold = mesherParams().zThreshold;
    float maxZ = p1.z;
    maxZ = std::max(maxZ, p2.z);
    maxZ = std::max(maxZ, p3.z);
    float minZ = p1.z;
    minZ = std::min(minZ, p2.z);
    minZ = std::min(minZ, p3.z);
    if (maxZ - minZ > zThreshold)
        return true;

    const double sideLengthThreshold3D = mesherParams().sideLengthThreshold3D;
    if (cv::norm(p2 - p1) > sideLengthThreshold3D) return true;
    if (cv::norm(p3 - p2) > sideLengthThreshold3D) return true;
    if (cv::norm(p1 - p3) > sideLengthThreshold3D) return true;
    return false;
}

}


Mesher::Mesher(FrameQueue &inputQueue, MeshFrameProducer &output, CancellationToken &cancellationToken)
    : FrameConsumer(inputQueue, cancellationToken)
    , output(output)
{
}

void Mesher::init()
{
    TLOG(INFO);
    const SensorManager &sensorManager = appState().getSensorManager();
    while (!cancel && !sensorManager.isInitialized())
        std::this_thread::sleep_for(30ms);

    ColorDataFormat colorFormat;
    DepthDataFormat depthFormat;
    sensorManager.getColorParams(colorCam, colorFormat);
    sensorManager.getDepthParams(depthCam, depthFormat);
    calibration = sensorManager.getCalibration();

    scale = float(targetScreenWidth) / depthCam.w;
    TLOG(INFO) << "Scale input depth by a factor of: " << scale;
    depthCam.scale(scale);
}

void Mesher::fillPoints(const cv::Mat &depth, std::vector<PointIJ> &points, std::vector<cv::Point3f> &cloud) const
{
    for (int i = 0; i < depth.rows; ++i)
    {
        const short scaleI = short(scale * i);
        for (int j = 0; j < depth.cols; ++j)
        {
            const uint16_t d = depth.at<uint16_t>(i, j);
            if (d > 0 && points.size() < std::numeric_limits<short>::max() - 10)
            {
                const short scaleJ = short(scale * j);
                points.emplace_back(scaleI, scaleJ);
                cloud.emplace_back(project2dPointTo3d(scaleI, scaleJ, d, depthCam));
            }
        }
    }
}

void Mesher::fillPoints(const std::vector<cv::Point3f> &frameCloud, std::vector<PointIJ> &points, std::vector<cv::Point3f> &cloud) const
{
    int iImg, jImg;
    uint16_t d;
    for (const auto &p : frameCloud)
        if (project3dPointTo2d(p, depthCam, iImg, jImg, d))
        {
            points.emplace_back(iImg, jImg);
            cloud.emplace_back(p);
        }
}

void Mesher::fillDataArrayMode(MeshFrame &frame, Triangle *triangles, int numTriangles, const std::vector<PointIJ> &points)
{
    frame.triangles3D.resize(numTriangles);
    frame.trianglesUv.resize(numTriangles);
    frame.trianglesNormals.resize(numTriangles);

    const bool needNormals = frame.frame2D->color.empty();

    int j = 0;
    for (int i = 0; i < numTriangles; ++i)
    {
        const Triangle &t = triangles[i];
        if (!skipFiltering && filterTriangle2D(points[t.p1], points[t.p2], points[t.p3]))
            continue;

        Triangle3D &t3d = frame.triangles3D[j];
        TriangleUV &tuv = frame.trianglesUv[j];
        Triangle3D &tn = frame.trianglesNormals[j];
        t3d.p1 = frame.cloud[t.p1];
        t3d.p2 = frame.cloud[t.p2];
        t3d.p3 = frame.cloud[t.p3];
        tuv.p1 = frame.uv[t.p1];
        tuv.p2 = frame.uv[t.p2];
        tuv.p3 = frame.uv[t.p3];

        if (!skipFiltering && filterTriangle3D(t3d.p1, t3d.p2, t3d.p3))
            continue;

        if (needNormals)
        {
            const cv::Point3f n = triNormal(t3d.p1, t3d.p2, t3d.p3);
            tn.p1 = tn.p2 = tn.p3 = n;
        }

        ++j;
    }

    frame.num3DTriangles = j;
}

void Mesher::fillDataIndexedMode(MeshFrame &frame, Triangle *triangles, int numTriangles, const std::vector<PointIJ> &points)
{
    const bool needNormals = frame.frame2D->color.empty();
    frame.normals.resize(frame.cloud.size());

    for (int i = 0; i < numTriangles; ++i)
    {
        const Triangle &t = triangles[i];
        if (!skipFiltering && filterTriangle2D(points[t.p1], points[t.p2], points[t.p3]))
            continue;

        const auto &p1 = frame.cloud[t.p1];
        const auto &p2 = frame.cloud[t.p2];
        const auto &p3 = frame.cloud[t.p3];
        if (!skipFiltering && filterTriangle3D(p1, p2, p3))
            continue;

        frame.triangles.emplace_back(t);

        if (needNormals)
            frame.normals[t.p1] = frame.normals[t.p2] = frame.normals[t.p3] = triNormal(p1, p2, p3);
    }
}

void Mesher::process(std::shared_ptr<Frame> &frame2D)
{
    tprof().startTimer("mesher_frame");
    auto meshFrame = std::make_shared<MeshFrame>();
    meshFrame->frame2D = frame2D;

    auto &cloud = meshFrame->cloud;
    auto &uv = meshFrame->uv;
    std::vector<PointIJ> points;

    if (!frame2D->depth.empty())
        fillPoints(frame2D->depth, points, cloud);
    else
        fillPoints(frame2D->cloud, points, cloud);

    tprof().startTimer("triangulation");
    std::vector<short> indexMap(points.size());
    delaunay(points, indexMap);
    delaunay.generateTriangles();

    Triangle *triangles = nullptr;
    int numTriangles = 0;
    delaunay.getTriangles(triangles, numTriangles);

    std::vector<cv::Point3f> sortedCloud(points.size());
    for (size_t i = 0; i < sortedCloud.size(); ++i)
        sortedCloud[i] = cloud[indexMap[i]];
    cloud.swap(sortedCloud);
    tprof().stopTimer("triangulation");

    // generate uv coordinates by reprojecting 3D points onto color image plane
    if (!frame2D->color.empty())
    {
        int iImg, jImg;
        uint16_t d;
        const cv::Point3f *translation = reinterpret_cast<cv::Point3f *>(calibration.tvec.data);
        for (size_t i = 0; i < cloud.size(); ++i)
        {
            float u = 0, v = 0;
            const cv::Point3f pointColorSpace = cloud[i] + *translation;
            if (project3dPointTo2d(pointColorSpace, colorCam, iImg, jImg, d))
            {
                // u - horizontal texture coordinate, v - vertical
                u = float(jImg) / colorCam.w;
                v = float(iImg) / colorCam.h;
            }

            uv.emplace_back(u, v);
        }
    }

    tprof().startTimer("meshing");

    meshFrame->indexedMode = true;
    if (meshFrame->indexedMode)
        fillDataIndexedMode(*meshFrame, triangles, numTriangles, points);
    else
        fillDataArrayMode(*meshFrame, triangles, numTriangles, points);

    tprof().stopTimer("meshing");
    tprof().stopTimer("mesher_frame");

    output.produce(meshFrame);
}
