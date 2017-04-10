#include <memory>
#include <fstream>

#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable: 4309)  // truncation of const value
#endif
#include <pxcsensemanager.h>
#ifdef _MSC_VER
    #pragma warning(pop)
#endif

#include <util/tiny_logger.hpp>


int main()
{
    PXCSenseManager *senseManager = PXCSenseManager::CreateInstance();

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
    senseManager->EnableStream(Intel::RealSense::StreamType::STREAM_TYPE_COLOR, 1920, 1080, 30, PXCCapture::Device::STREAM_OPTION_STRONG_STREAM_SYNC);
    senseManager->EnableStream(Intel::RealSense::StreamType::STREAM_TYPE_DEPTH, 480, 360, 30);  // also supports 628x468

    senseManager->Init();

    TLOG(INFO) << "Setting device properties...";
    auto device = senseManager->QueryCaptureManager()->QueryDevice();
    device->SetColorAutoExposure(true);
    device->SetColorAutoWhiteBalance(true);
    device->SetDSLeftRightAutoExposure(true);

    int numFrames = 0;

    while (senseManager->AcquireFrame(true, 1000) >= PXC_STATUS_NO_ERROR)
    {
        PXCCapture::Sample *sample = senseManager->QuerySample();
        if (!sample)
        {
            TLOG(ERROR) << "Sample is null";
            continue;
        }

        if (!sample->color)
        {
            TLOG(ERROR) << "Color is null";
            continue;
        }

        if (!sample->depth)
        {
            TLOG(ERROR) << "Depth is null";
            continue;
        }


        const auto colorInfo = sample->color->QueryInfo(), depthInfo = sample->depth->QueryInfo();
        senseManager->ReleaseFrame();
        ++numFrames;
        TLOG(INFO) << "Captured color frame #" << numFrames << " " << colorInfo.format << " " << colorInfo.width << " " << colorInfo.height << " " << colorInfo.reserved;
        TLOG(INFO) << "Captured depth frame #" << numFrames << " " << depthInfo.format << " " << depthInfo.width << " " << depthInfo.height << " " << depthInfo.reserved;
    }

    senseManager->Release();

    return EXIT_SUCCESS;
}
