#pragma once

#include <fstream>


class DatasetInput
{
public:
    DatasetInput(const std::string &path);

private:
    std::ifstream in;
};
