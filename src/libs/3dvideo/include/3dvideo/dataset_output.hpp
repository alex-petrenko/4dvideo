#pragma once

#include <fstream>

#include <util/enum.hpp>

#include <3dvideo/frame.hpp>
#include <3dvideo/format.hpp>


class DatasetOutput
{
public:
    DatasetOutput(const std::string &path);

    Status writeHeader();
    Status writeFrame(const Frame &frame);

private:
    template<typename T>
    void binWrite(T val)
    {
        out.write((char *)&val, sizeof(val));
    }

    template<typename T>
    void writeField(Field field, T value)
    {
        binWrite(field);
        binWrite(value);
    }

    /// T is expected to be either char or uchar or uint8_t, etc. (helps to reduce amount of boilerplate casts)
    template<typename T>
    void writeField(Field field, const T *data, size_t size)
    {
        static_assert(sizeof(T) == 1, "1-byte type expected");
        binWrite(field);
        out.write((char *)data, size);
    }

private:
    std::ofstream out;
};
