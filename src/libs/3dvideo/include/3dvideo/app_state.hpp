#pragma once

#include <atomic>

#include <3dvideo/sensor_manager.hpp>


class AppState
{
public:
    static AppState & instance();

    void startGrabbing();
    bool isGrabbingStarted() const;

    void stopGrabbing();
    bool isGrabbingStopped();

    void initializeSensorManager(const CameraParams &color, const CameraParams &depth);

private:
    AppState() = default;
    AppState(const AppState &) = delete;
    void operator=(const AppState &) = delete;

private:
    std::atomic_bool start = false, stop = false;

    SensorManager sensorManager;
};

AppState & appState();
