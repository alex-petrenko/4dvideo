#include <4d/sensor_manager.hpp>


void SensorManager::setColorParams(const CameraParams &colorCamera, ColorDataFormat format)
{
    color = colorCamera;
    colorFormat = format;
}

void SensorManager::setDepthParams(const CameraParams &depthCamera, DepthDataFormat format)
{
    depth = depthCamera;
    depthFormat = format;
}

void SensorManager::setCalibration(const Calibration &calib)
{
    calibration = calib;
}

void SensorManager::setInitialized(bool value)
{
    initialized = value;
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

Calibration SensorManager::getCalibration() const
{
    return calibration;
}
