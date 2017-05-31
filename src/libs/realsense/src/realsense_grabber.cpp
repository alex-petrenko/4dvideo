#include <opencv2/core.hpp>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4309)  // truncation of const value
#endif
#include <pxcsensemanager.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <util/camera.hpp>
#include <util/tiny_logger.hpp>

#include <realsense/realsense_grabber.hpp>

#include <3dvideo/app_state.hpp>


using namespace Intel;
using namespace std::chrono_literals;


namespace
{

// constants
#if defined(COLOR_1080P)
    constexpr int colorW = 1920, colorH = 1080;
#else
    constexpr int colorW = 1280, colorH = 720;
#endif

#if defined(DEPTH_360P)
    constexpr int depthW = 480, depthH = 360;
#else
    constexpr int depthW = 320, depthH = 240;
#endif

constexpr int fps = 30;

}


struct RealsenseGrabber::RealsenseGrabberImpl
{
    PXCSenseManager *senseManager;
};


RealsenseGrabber::RealsenseGrabber(const CancellationToken &cancellationToken)
    : Producer(cancellationToken)
{
    data = std::make_unique<RealsenseGrabberImpl>();
}

RealsenseGrabber::~RealsenseGrabber()
{
    if (data->senseManager)
        data->senseManager->Release();
}

void RealsenseGrabber::init()
{
    data->senseManager = PXCSenseManager::CreateInstance();
    auto senseManager = data->senseManager;

    if (!senseManager)
    {
        TLOG(ERROR) << "Could not start initialization";
        return;
    }

    auto session = senseManager->QuerySession();
    auto version = session->QueryVersion();
    TLOG(INFO) << "RealSense capture session " << version.major << "." << version.minor;

    PXCSession::ImplDesc description = {};
    description.group = PXCSession::IMPL_GROUP_SENSOR;
    description.subgroup = PXCSession::IMPL_SUBGROUP_VIDEO_CAPTURE;

    TLOG(INFO) << "Loop over available capture modules...";
    for (int moduleIdx = 0; ; ++moduleIdx)
    {
        PXCSession::ImplDesc moduleDescription;
        if (session->QueryImpl(&description, moduleIdx, &moduleDescription) < PXC_STATUS_NO_ERROR)
            break;

        PXCCapture *capture = nullptr;
        auto status = session->CreateImpl<PXCCapture>(&moduleDescription, &capture);
        if (status < PXC_STATUS_NO_ERROR)
        {
            TLOG(ERROR) << "Unable to get information about capture module, status: " << status;
            continue;
        }

        // print out all device information
        int numDevices = 0;
        for (int deviceIdx = 0; ; deviceIdx++)
        {
            PXCCapture::DeviceInfo deviceInfo;
            status = capture->QueryDeviceInfo(deviceIdx, &deviceInfo);
            if (status < PXC_STATUS_NO_ERROR)
                break;

            TLOG(INFO) << "Device #" << deviceIdx << " " << deviceInfo.name << " " << deviceInfo.model;
            ++numDevices;
        }

        if (numDevices > 0)
            TLOG(INFO) << "Module #" << moduleIdx << " " << moduleDescription.friendlyName;

        capture->Release();
    }

    TLOG(INFO) << "Enabling streams...";
    senseManager->EnableStream(RealSense::StreamType::STREAM_TYPE_COLOR, colorW, colorH, float(fps), PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);
    senseManager->EnableStream(RealSense::StreamType::STREAM_TYPE_DEPTH, depthW, depthH, float(fps));  // also supports 628x468

    senseManager->Init();

    TLOG(INFO) << "Setting device properties...";
    auto captureManager = senseManager->QueryCaptureManager();
    if (!captureManager)
        return;
    auto device = captureManager->QueryDevice();
    if (!device)
        return;

    for (int profileIdx = 0; ; ++profileIdx)
    {
        PXCCapture::Device::StreamProfileSet profiles;
        const auto status = device->QueryStreamProfileSet(PXCCapture::STREAM_TYPE_DEPTH | PXCCapture::STREAM_TYPE_COLOR, profileIdx, &profiles);
        if (status < PXC_STATUS_NO_ERROR)
            TLOG(FATAL) << "Stream parameters do not match any profile!";

        const int dw = profiles.depth.imageInfo.width, dh = profiles.depth.imageInfo.height;
        const int cw = profiles.color.imageInfo.width, ch = profiles.color.imageInfo.height;
        TLOG(INFO) << "Profile idx: " << profileIdx << " depth: " << dw << "x" << dh << " color: " << cw << "x" << ch;
        if (dw == depthW && dh == depthH && cw == colorW && ch == colorH && fps == int(profiles.depth.frameRate.max + 0.5f))
        {
            TLOG(INFO) << "Found matching profile!";
            break;
        }
    }

    device->SetColorAutoExposure(true);
    device->SetColorAutoWhiteBalance(true);
    device->SetDSLeftRightAutoExposure(true);

    const auto colorF = device->QueryColorFocalLength().x;  // assuming fx and fy are roughly the same
    const auto colorCenter = device->QueryColorPrincipalPoint();
    const auto depthF = device->QueryDepthFocalLength().x;
    const auto depthCenter = device->QueryDepthPrincipalPoint();

    // find rotation and translation transformation between color and depth cameras
    PXCProjection *projection = device->CreateProjection();
    PXCCalibration *calib = projection->QueryInstance<PXCCalibration>();
    RealSense::StreamCalibration colorStreamCalibration;
    RealSense::StreamTransform extrinsics;
    calib->QueryStreamProjectionParameters(RealSense::StreamType::STREAM_TYPE_COLOR, &colorStreamCalibration, &extrinsics);
    projection->Release();

    CameraParams colorCamera(colorF, colorCenter.x, colorCenter.y, colorW, colorH);
    CameraParams depthCamera(depthF, depthCenter.x, depthCenter.y, depthW, depthH);
    Calibration calibration;
    calibration.rmat = cv::Mat(3, 3, CV_32F, extrinsics.rotation);
    calibration.tvec = cv::Mat(3, 1, CV_32F, extrinsics.translation);
    calibration.tvec *= 0.001f;  // convert millimeters to meters

    SensorManager &sensorManager = appState().getSensorManager();
    sensorManager.setColorParams(colorCamera, ColorDataFormat::BGR);
    sensorManager.setDepthParams(depthCamera, DepthDataFormat::UNSIGNED_16BIT_MM);
    sensorManager.setCalibration(calibration);

    TLOG(INFO) << "Calibration parameters. Translation:";
    TLOG(INFO) << extrinsics.translation[0] << " " << extrinsics.translation[1] << " " << extrinsics.translation[2];
    TLOG(INFO) << "Rotation:";
    for (int i = 0; i < 3; ++i)
        TLOG(INFO) << extrinsics.rotation[i][0] << " " << extrinsics.rotation[i][1] << " " << extrinsics.rotation[i][2];

    sensorManager.setInitialized();
}

void RealsenseGrabber::run()
{
    while (!cancel && !appState().isGrabbingStarted())
        std::this_thread::sleep_for(30ms);

    auto senseManager = data->senseManager;

    int numFrames = 0;

    RealSense::Status status = PXC_STATUS_NO_ERROR;
    while (!cancel && !appState().isGrabbingStopped())
    {
        status = senseManager->AcquireFrame(true, 1000);
        if (status < PXC_STATUS_NO_ERROR)
            break;

        auto sample = senseManager->QuerySample();
        
        if (!sample)
        {
            TLOG(ERROR) << "Sample is null";
            continue;
        }

        auto color = sample->color, depth = sample->depth;
        if (!color)
        {
            TLOG(ERROR) << "Color is null";
            continue;
        }
        if (!depth)
        {
            TLOG(ERROR) << "Depth is null";
            continue;
        }

        auto frame = std::make_shared<Frame>();

        const auto colorInfo = color->QueryInfo(), depthInfo = depth->QueryInfo();
        const auto colorTimestamp = color->QueryTimeStamp(), depthTimestamp = depth->QueryTimeStamp();
        const auto colorTimestampUs = colorTimestamp / 10, depthTimestampUs = depthTimestamp / 10;
        ++numFrames;
        TLOG(INFO) << "Captured color frame #" << numFrames << " " << colorInfo.format << " " << colorInfo.width << " " << colorInfo.height << " " << colorInfo.reserved;
        TLOG(INFO) << "Captured depth frame #" << numFrames << " " << depthInfo.format << " " << depthInfo.width << " " << depthInfo.height << " " << depthInfo.reserved;

        RealSense::Image::ImageData colorData, depthData;

        auto accessStatus = color->AcquireAccess(RealSense::Image::ACCESS_READ, RealSense::Image::PIXEL_FORMAT_BGR, &colorData);
        if (accessStatus < PXC_STATUS_NO_ERROR)
        {
            TLOG(ERROR) << "Could not acquire access to color buffer for frame #" << numFrames << ", status is: " << accessStatus;
            continue;
        }

        cv::Mat(colorInfo.height, colorInfo.width, CV_8UC3, colorData.planes[0]).copyTo(frame->color);
        color->ReleaseAccess(&colorData);

        accessStatus = depth->AcquireAccess(RealSense::Image::ACCESS_READ, RealSense::Image::PIXEL_FORMAT_DEPTH, &depthData);
        if (accessStatus < PXC_STATUS_NO_ERROR)
        {
            TLOG(ERROR) << "Could not acquire access to depth buffer for frame #" << numFrames << ", status is: " << accessStatus;
            continue;
        }

        cv::Mat(depthInfo.height, depthInfo.width, CV_16UC1, depthData.planes[0]).copyTo(frame->depth);
        depth->ReleaseAccess(&depthData);

        senseManager->ReleaseFrame();

        frame->frameNumber = numFrames;
        frame->cTimestamp = colorTimestampUs, frame->dTimestamp = depthTimestampUs;
        produce(frame);
    }

    TLOG(INFO) << "Grabbing thread has finished, last status: " << status;
}
