#pragma once

#include <atomic>

#include <util/camera.hpp>

#include <4d/format.hpp>


class SensorManager
{
public:
    void setInitialized(bool initialized = true);
    bool isInitialized() const;

    void setColorParams(const CameraParams &colorCamera, ColorDataFormat format);
    void setDepthParams(const CameraParams &depthCamera, DepthDataFormat format);
    void setCalibration(const Calibration &calib);

    void getColorParams(CameraParams &colorCamera, ColorDataFormat &format) const;
    void getDepthParams(CameraParams &depthCamera, DepthDataFormat &format) const;
    Calibration getCalibration() const;

private:
    std::atomic_bool initialized;

    CameraParams color, depth;
    Calibration calibration;
    ColorDataFormat colorFormat;
    DepthDataFormat depthFormat;
};
