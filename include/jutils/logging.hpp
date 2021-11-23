#ifndef JUTILS_LOGGING_H_
#define JUTILS_LOGGING_H_

#include <iostream>

#define JLOG_CRITICAL(...)   jutils::logging::log(jutils::logging::level::CRITICAL,   ##__VA_ARGS__)
#define JLOG_ERROR(...)      jutils::logging::log(jutils::logging::level::ERROR,      ##__VA_ARGS__)
#define JLOG_WARN(...)       jutils::logging::log(jutils::logging::level::WARN,       ##__VA_ARGS__)
#define JLOG_INFO(...)       jutils::logging::log(jutils::logging::level::INFO,       ##__VA_ARGS__)
#define JLOG_DEBUG(...)      jutils::logging::log(jutils::logging::level::DEBUG,      ##__VA_ARGS__)

#define JLOGN_CRITICAL(name,...)   jutils::logging::log(name,jutils::logging::level::CRITICAL,   ##__VA_ARGS__)
#define JLOGN_ERROR(name,...)      jutils::logging::log(name,jutils::logging::level::ERROR,      ##__VA_ARGS__)
#define JLOGN_WARN(name,...)       jutils::logging::log(name,jutils::logging::level::WARN,       ##__VA_ARGS__)
#define JLOGN_INFO(name,...)       jutils::logging::log(name,jutils::logging::level::INFO,       ##__VA_ARGS__)
#define JLOGN_DEBUG(name,...)      jutils::logging::log(name,jutils::logging::level::DEBUG,      ##__VA_ARGS__)

namespace jutils {
namespace logging {

enum class level : int {
    CRITICAL = 4,
    ERROR = 3,
    WARN = 2,
    INFO = 1,
    DEBUG = 0
};

class Consumer {
public:
    virtual void log(logging::level level, std::string message) = 0;
    virtual void name_log(std::string name, logging::level level, std::string message) = 0;
    virtual void setLevel(logging::level level) = 0;
};

inline void log(level level, std::string message) {
    (void) level;

    std::printf("%s\n", message.c_str());
}

inline void name_log(std::string name, level level, std::string message) {
    (void) name;
    (void) level;

    std::printf("%s\n", message.c_str());
}

} /* namespace logging */






} /* namespace jutils */


#endif /* JUTILS_LOGGING_H_ */