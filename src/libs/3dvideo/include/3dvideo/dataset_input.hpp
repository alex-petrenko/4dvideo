#pragma once

#include <fstream>

#include <util/enum.hpp>


class DatasetInput
{
public:
    DatasetInput(const std::string &path);

    Status readHeader();
    Status readFrame();

private:
    template<typename T>
    void binRead(T &val)
    {
        in.read((char *)&val, sizeof(val));
    }

private:
    std::ifstream in;
};
