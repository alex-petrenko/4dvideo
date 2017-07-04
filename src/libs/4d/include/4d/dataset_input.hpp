#pragma once

#include <map>
#include <fstream>
#include <functional>

#include <util/enum.hpp>
#include <util/camera.hpp>

#include <4d/frame.hpp>
#include <4d/format.hpp>


struct DatasetMetadata
{
    uint32_t formatVersion;
    CameraParams color, depth;
    ColorDataFormat colorFormat;
    DepthDataFormat depthFormat;
    Calibration calibration;
};

class DatasetInput
{
    typedef std::function<bool()> FieldParser;
    typedef std::function<bool(Frame &)> FrameFieldParser;

public:
    DatasetInput(const std::string &path, bool readColor);
    ~DatasetInput();

    Status readHeader();
    Status readFrame(Frame &frame);

    DatasetMetadata getMetadata() const;

    bool finished() const;

private:
    template<typename T>
    bool binRead(T &value)
    {
        return bool(in.read((char *)&value, sizeof(value)));
    }

    template<typename T, typename... Args>
    bool binRead(T &value, Args&&... args)
    {
        return binRead(value) && binRead(std::forward<Args>(args)...);
    }

    bool readMetadataField(Field &field);
    bool readFrameField(Frame &frame, Field &field);

private:
    bool withColor;
    bool isFinished = false;
    std::ifstream in;
    std::map<Field, FieldParser> metadataParsers;
    std::map<Field, FrameFieldParser> frameParsers;
    std::map<ColorDataFormat, FrameFieldParser> colorReaders;

    DatasetMetadata meta;
};
