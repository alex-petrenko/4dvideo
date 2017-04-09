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
    /*PXCSession *session = PXCSession::CreateInstance();
    PXCSession::ImplVersion ver = session->QueryVersion();
    TLOG(INFO) << ver.major << "." << ver.minor;
    session->Release();*/

    PXCSenseManager *senseManager = PXCSenseManager::CreateInstance();
    senseManager->EnableStream(Intel::RealSense::StreamType::STREAM_TYPE_COLOR, 1920, 1080, 30.0f);
    senseManager->EnableStream(Intel::RealSense::StreamType::STREAM_TYPE_DEPTH, 628, 468, 30.0f);

    senseManager->Init();

    int numFrames = 0;

    while (senseManager->AcquireFrame(true) >= PXC_STATUS_NO_ERROR)
    {
        PXCCapture::Sample *sample = senseManager->QuerySample();
        const auto colorInfo = sample->color->QueryInfo(), depthInfo = sample->depth->QueryInfo();
        senseManager->ReleaseFrame();
        ++numFrames;
        TLOG(INFO) << "Captured color frame #" << numFrames << " " << colorInfo.format << " " << colorInfo.width << " " << colorInfo.height << " " << colorInfo.reserved;
        TLOG(INFO) << "Captured depth frame #" << numFrames << " " << depthInfo.format << " " << depthInfo.width << " " << depthInfo.height << " " << depthInfo.reserved;
    }

    senseManager->Release();

    return EXIT_SUCCESS;
}
