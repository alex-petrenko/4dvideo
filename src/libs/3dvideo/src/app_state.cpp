#include <util/tiny_logger.hpp>

#include <3dvideo/app_state.hpp>


AppState & AppState::instance()
{
    static AppState instance;
    return instance;
}

void AppState::startGrabbing()
{
    TLOG(INFO);
    start = true;
}

bool AppState::isGrabbingStarted() const
{
    return start.load();
}

void AppState::stopGrabbing()
{
    TLOG(INFO);
    stop = true;
}

bool AppState::isGrabbingStopped()
{
    return stop.load();
}

AppState & appState()
{
    return AppState::instance();
}
