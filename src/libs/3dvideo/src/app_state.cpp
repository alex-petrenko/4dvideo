#include <util/tiny_logger.hpp>

#include <3dvideo/app_state.hpp>


AppState & AppState::instance()
{
    static AppState state;
    return state;
}

void AppState::reset()
{
    TLOG(INFO);
    grabbingStarted = capturingStopped = false;
    sensorManager.setInitialized(false);
}

void AppState::startCapturing()
{
    TLOG(INFO);
    capturingStarted = true;
}

bool AppState::isCapturingStarted() const
{
    return capturingStarted.load();
}

void AppState::startGrabbing()
{
    TLOG(INFO);
    grabbingStarted = true;
}

bool AppState::isGrabbingStarted() const
{
    return grabbingStarted.load();
}

void AppState::stopCapturing()
{
    TLOG(INFO);
    capturingStopped = true;
}

bool AppState::isCapturingStopped()
{
    return capturingStopped.load();
}

SensorManager & AppState::getSensorManager()
{
    return sensorManager;
}

AppState & appState()
{
    return AppState::instance();
}
