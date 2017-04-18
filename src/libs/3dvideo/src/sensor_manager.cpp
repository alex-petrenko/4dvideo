#include <3dvideo/sensor_manager.hpp>


void SensorManager::initialize(const CameraParams &colorCamera, const CameraParams &depthCamera)
{
    color = colorCamera;
    depth = depthCamera;
    initialized = true;
}

bool SensorManager::isInitialized() const
{
    return initialized.load();
}
