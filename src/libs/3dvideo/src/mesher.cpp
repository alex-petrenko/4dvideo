#include <thread>

#include <util/tiny_logger.hpp>
#include <util/tiny_profiler.hpp>

#include <3dvideo/mesher.hpp>
#include <3dvideo/player.hpp>
#include <3dvideo/app_state.hpp>


using namespace std::chrono_literals;


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
    const uint16_t minDepth = 200, maxDepth = 1900;
    for (int i = 0; i < depth.rows; i += 1)
    {
        const short scaleI = short(scale * i);
        for (int j = 0; j < depth.cols; j += 1)
        {
            const uint16_t d = depth.at<uint16_t>(i, j);
            if (d > minDepth && d < maxDepth && points.size() < std::numeric_limits<short>::max())
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

void Mesher::fillDataArrayMode(MeshFrame &frame, Triangle *triangles, int numTriangles)
{
    const float sideLengthThreshold = 0.075f;  // in meters
    const float zThreshold = 0.06f;

    frame.triangles3D.resize(numTriangles);
    frame.trianglesUv.resize(numTriangles);
    frame.trianglesNormals.resize(numTriangles);

    int j = 0;
    for (int i = 0; i < numTriangles; ++i)
    {
        Triangle3D &t3d = frame.triangles3D[j];
        TriangleUV &tuv = frame.trianglesUv[j];
        Triangle3D &tn = frame.trianglesNormals[j];
        const Triangle &t = triangles[i];

        t3d.p1 = frame.cloud[t.p1];
        t3d.p2 = frame.cloud[t.p2];
        t3d.p3 = frame.cloud[t.p3];
        tuv.p1 = frame.uv[t.p1];
        tuv.p2 = frame.uv[t.p2];
        tuv.p3 = frame.uv[t.p3];

        float maxZ = t3d.p1.z;
        maxZ = std::max(maxZ, t3d.p2.z);
        maxZ = std::max(maxZ, t3d.p3.z);
        float minZ = t3d.p1.z;
        minZ = std::min(minZ, t3d.p2.z);
        minZ = std::min(minZ, t3d.p3.z);
        if (maxZ - minZ > zThreshold)
            continue;

        const float a = float(cv::norm(t3d.a()));
        if (a > sideLengthThreshold) continue;
        const float b = float(cv::norm(t3d.b()));
        if (b > sideLengthThreshold) continue;
        const float c = float(cv::norm(t3d.c()));
        if (c > sideLengthThreshold) continue;

        // TODO: need normals only when there's no texture
        const bool needNormals = false;
        if (needNormals)
        {
            const cv::Point3f n = triNormal(t3d.p1, t3d.p2, t3d.p3);
            // all three points have same normal TODO: optimize
            tn.p1 = tn.p2 = tn.p3 = n;
        }

        ++j;
    }

    frame.num3DTriangles = j;
}

void Mesher::fillDataIndexedMode(MeshFrame &frame, Triangle *triangles, int numTriangles)
{
    frame.triangles = std::vector<Triangle>(triangles, triangles + numTriangles);
    frame.normals.resize(frame.cloud.size());
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
        fillDataIndexedMode(*meshFrame, triangles, numTriangles);
    else
        fillDataArrayMode(*meshFrame, triangles, numTriangles);

    tprof().stopTimer("meshing");
    tprof().stopTimer("mesher_frame");

    output.produce(meshFrame);
}
