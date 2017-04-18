#include <3dvideo/sensor_manager.hpp>


void SensorManager::setColorParams(const CameraParams &colorCamera, ColorDataFormat format)
{
    color = colorCamera;
    colorFormat = format;
}

void SensorManager::setDepthParams(const CameraParams & depthCamera, DepthDataFormat format)
{
    depth = depthCamera;
    depthFormat = format;
}

void SensorManager::setInitialized()
{
    initialized = true;
}

bool SensorManager::isInitialized() const
{
    return initialized.load();
}

void SensorManager::getColorParams(CameraParams &colorCamera, ColorDataFormat &format) const
{
    colorCamera = color, format = colorFormat;
}

void SensorManager::getDepthParams(CameraParams &depthCamera, DepthDataFormat &format) const
{
    depthCamera = depth, format = depthFormat;
}
