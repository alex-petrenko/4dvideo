#pragma once

#include <string>
#include <sstream>


inline char pathDelim()
{
#if defined(_WIN32)
    return '\\';
#else
    return '/';
#endif
}

template<typename T>
void pathJoinHelper(std::ostringstream &stream, const T &t)
{
    stream << t;
}

template<typename T, typename... Args>
void pathJoinHelper(std::ostringstream &stream, const T &t, Args... args)
{
    stream << t << pathDelim();
    pathJoinHelper(stream, std::forward<Args>(args)...);
}

template<typename... Args>
std::string pathJoin(Args... args)
{
    std::ostringstream stream;
    pathJoinHelper(stream, std::forward<Args>(args)...);
    return stream.str();
}
