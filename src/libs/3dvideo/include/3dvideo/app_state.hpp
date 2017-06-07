#pragma once

#include <atomic>

#include <3dvideo/sensor_manager.hpp>


class AppState
{
public:
    static AppState & instance();

    void reset();

    void startCapturing();
    bool isCapturingStarted() const;

    void startGrabbing();
    bool isGrabbingStarted() const;

    void stopCapturing();
    bool isCapturingStopped();

    SensorManager & getSensorManager();

private:
    AppState() = default;
    AppState(const AppState &) = delete;
    void operator=(const AppState &) = delete;

private:
    std::atomic_bool capturingStarted = false, grabbingStarted = false, capturingStopped = false;

    SensorManager sensorManager;
};

AppState & appState();
