#pragma once

#include <atomic>

#include <3dvideo/format.hpp>

#include <util/camera.hpp>


class SensorManager
{
public:
    void setInitialized(bool initialized = true);
    bool isInitialized() const;
    
    void setColorParams(const CameraParams &colorCamera, ColorDataFormat format);
    void setDepthParams(const CameraParams &depthCamera, DepthDataFormat format);

    void getColorParams(CameraParams &colorCamera, ColorDataFormat &format) const;
    void getDepthParams(CameraParams &depthCamera, DepthDataFormat &format) const;

private:
    std::atomic_bool initialized;

    CameraParams color, depth;
    ColorDataFormat colorFormat;
    DepthDataFormat depthFormat;
};
