#include <fstream>

#include <pxcsession.h>

#include <util/tiny_logger.hpp>


int main()
{
    PXCSession *session = PXCSession::CreateInstance();
    PXCSession::ImplVersion ver = session->QueryVersion();
    TLOG(INFO) << ver.major << "." << ver.minor;
    session->Release();

    return EXIT_SUCCESS;
}
