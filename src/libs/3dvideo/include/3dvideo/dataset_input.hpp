#pragma once

#include <map>
#include <fstream>
#include <functional>

#include <util/enum.hpp>
#include <util/camera.hpp>

#include <3dvideo/frame.hpp>
#include <3dvideo/format.hpp>


struct DatasetMetadata
{
    uint32_t formatVersion;
    CameraParams color, depth;
    ColorDataFormat colorFormat;
    DepthDataFormat depthFormat;
};

class DatasetInput
{
    typedef std::function<bool()> FieldParser;
    typedef std::function<bool(Frame &)> FrameFieldParser;

public:
    DatasetInput(const std::string &path);

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

    template<typename T>
    bool readField(Field field, T &value)
    {
        Field f;
        bool ok = binRead(f);
        if (!ok || f != field)
            return false;

        ok = binRead(value);
        return ok;
    }

    bool readMetadataField(Field &field);
    bool readFrameField(Frame &frame, Field &field);

private:
    bool isFinished = false;
    std::ifstream in;
    std::map<Field, FieldParser> metadataParsers;
    std::map<Field, FrameFieldParser> frameParsers;

    DatasetMetadata meta;
};
