#include <util/filesystem_utils.hpp>


size_t readAllBytes(std::ifstream &stream, std::vector<char> &buffer)
{
    stream.seekg(0, std::ios::end);
    const auto size = stream.tellg();
    stream.seekg(0);
    buffer.resize(size);
    if (stream.read(buffer.data(), size))
        return size;
    else
        return 0;
}
