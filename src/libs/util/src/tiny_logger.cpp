#include <chrono>
#include <iomanip>

#include <util/tiny_logger.hpp>
#include <util/filesystem_utils.hpp>


using namespace std::chrono;


namespace
{

/// Everything with level higher than logLevel will be discarded.
const LogLevel logLevel = DEBUG;

const char * levelToStr(LogLevel level)
{
    switch (level)
    {
    case FATAL: return "FAT";
    case ERROR: return "ERR";
    case WARNING: return "WRN";
    case INFO: return "INF";
    case VERBOSE: return "VER";
    case DEBUG: return "DBG";
    default: return "";
    }
}

const char * constBasename(const char *filepath)
{
    const char *basename = strrchr(filepath, pathDelim());
    return basename ? basename + 1 : filepath;
}

void printTime(std::ostream &stream)
{
    char timeStr[1 << 6];
    const system_clock::time_point now = system_clock::now();
    const time_t timestamp = system_clock::to_time_t(now);
    strftime(timeStr, sizeof(timeStr), "%m-%d %T", std::localtime(&timestamp));

    const auto sinceEpoch = now.time_since_epoch();
    const system_clock::duration fraction = sinceEpoch - duration_cast<seconds>(sinceEpoch);
    const long long ms = duration_cast<milliseconds>(fraction).count();

    stream << timeStr << '.' << std::setw(3) << std::setfill('0') << ms << ' ';
}

}


LogMessage::LogMessage(LogLevel level, const char *file, int line, const char *func)
    : stream(nullptr)
{
    if (level > logLevel)
    {
        static NullStream nullStream;
        stream = &nullStream;
    }
    else
    {
        stream = &std::cout;

        *stream << "[";
        printTime(*stream);

        *stream << levelToStr(level) << ' ' << constBasename(file) << ':' << line;
        if (func)
        {
            *stream << ' ' << func;
            if (!strchr(func, '('))
                *stream << "()";
        }
        *stream << "] ";
    }
}

LogMessage::~LogMessage()
{
    if (stream)
        *stream << '\n';
}

std::ostream & LogMessage::operator()()
{
    return *stream;
}
