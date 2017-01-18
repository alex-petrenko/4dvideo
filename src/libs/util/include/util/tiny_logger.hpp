#pragma once

#include <vector>
#include <ostream>
#include <iostream>


enum LogLevel
{
    FATAL,
    ERROR,
    WARNING,
    INFO,
    VERBOSE,
    DEBUG,
};

class LogMessage
{
public:
    LogMessage(LogLevel level, const char *file, int line, const char *func);
    ~LogMessage();

    std::ostream & operator()();

private:
    std::ostream *stream;
};

class NullStream : public std::ostream
{
public:
    NullStream()
        : std::ostream(nullptr)
    {}
};

template<typename T>
inline NullStream & operator<<(NullStream &stream, const T &)
{
    return stream;
}


// helper macros

#define TLOG(level) LogMessage(level, __FILE__, __LINE__, __FUNCTION__)()

#define TLOG_IF(level, cond) !(cond) ? (void)0 : TLOG(level)


// various helper functions

template <typename T>
std::ostream & operator<<(std::ostream &stream, const std::vector<T> &vec)
{
    stream << '[';
    const auto end = std::end(vec);
    for (auto it = std::begin(vec); it != end; ++it)
    {
        stream << *it;
        if (it + 1 != end)
            stream << ", ";
    }
    stream << ']';
    return stream;
}