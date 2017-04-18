#pragma once

#include <atomic>

#include <util/camera.hpp>


class SensorManager
{
public:
    void initialize(const CameraParams &colorCamera, const CameraParams &depthCamera);
    bool isInitialized() const;

private:
    std::atomic_bool initialized = false;

    CameraParams color, depth;
};
