#pragma once

#include <fstream>

#include <util/enum.hpp>

#include <3dvideo/frame.hpp>
#include <3dvideo/format.hpp>
#include <3dvideo/sensor_manager.hpp>


class DatasetOutput
{
public:
    DatasetOutput(const std::string &path);

    Status writeHeader(const SensorManager &sensorManager);
    Status writeFrame(const Frame &frame);

private:
    template<typename T>
    void binWrite(T val)
    {
        out.write((const char *)&val, sizeof(val));
    }

    // specialization for cv::Mat
    template<>
    void binWrite(cv::Mat m)
    {
        out.write((const char *)m.data, m.total() * m.elemSize());
    }

    template<typename T, typename... Args>
    void binWrite(T val, Args&&... args)
    {
        binWrite(val), binWrite(std::forward<Args>(args)...);
    }

    template<typename T>
    void writeField(Field field, T value)
    {
        binWrite(field);
        binWrite(value);
    }

    void writeField(Field field, const char *data, size_t size)
    {
        binWrite(field);
        out.write(data, size);
    }

private:
    std::ofstream out;
};
