/**
 * @brief a very lightweight logging library that can handle different levels
 *  and also output to different consumers of the logs (eg: stdout vs file)
 * @date 11/23/2021
 * @author Joshua Spisak <jspisak@andrew.cmu.edu>
 * 
 * Copyright 2021 Joshua Spisak
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 **/
#ifndef JUTILS_LOGGING_HPP_
#define JUTILS_LOGGING_HPP_
#define JUTILS_LOGGING_VERSION_MAJOR 0
#define JUTILS_LOGGING_VERSION_MINOR 1

#include <sys/stat.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <map>

//! Simple message logging
#define JLOG_DIRE(...)      jutils::logging::log(jutils::logging::level::DIRE,   ##__VA_ARGS__)
#define JLOG_FAIL(...)      jutils::logging::log(jutils::logging::level::FAIL,   ##__VA_ARGS__)
#define JLOG_WARN(...)      jutils::logging::log(jutils::logging::level::WARN,   ##__VA_ARGS__)
#define JLOG_INFO(...)      jutils::logging::log(jutils::logging::level::INFO,   ##__VA_ARGS__)
#define JLOG_NOTE(...)      jutils::logging::log(jutils::logging::level::NOTE,   ##__VA_ARGS__)

//! Logging with named loggers
#define JLOGN_DIRE(name,...)   jutils::logging::log(name,jutils::logging::level::DIRE,   ##__VA_ARGS__)
#define JLOGN_FAIL(name,...)   jutils::logging::log(name,jutils::logging::level::FAIL,   ##__VA_ARGS__)
#define JLOGN_WARN(name,...)   jutils::logging::log(name,jutils::logging::level::WARN,   ##__VA_ARGS__)
#define JLOGN_INFO(name,...)   jutils::logging::log(name,jutils::logging::level::INFO,   ##__VA_ARGS__)
#define JLOGN_NOTE(name,...)   jutils::logging::log(name,jutils::logging::level::NOTE,   ##__VA_ARGS__)

namespace jutils {

//! formats strings given args (uses strings instead of printf)
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args ... ) + 1;
    auto buf = std::make_unique<char[]>( size_s );
    std::snprintf( buf.get(), size_s, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size_s - 1 );
}

//! Terminal Coloring
namespace term {
    //! Foreground Coloring
    namespace fg {
        __attribute__((unused)) static const char* def           = "\033[39m";
        __attribute__((unused)) static const char* black         = "\033[30m";
        __attribute__((unused)) static const char* red           = "\033[31m";
        __attribute__((unused)) static const char* green         = "\033[32m";
        __attribute__((unused)) static const char* yellow        = "\033[33m";
        __attribute__((unused)) static const char* blue          = "\033[34m";
        __attribute__((unused)) static const char* magenta       = "\033[35m";
        __attribute__((unused)) static const char* cyan          = "\033[36m";
        __attribute__((unused)) static const char* light_gray    = "\033[37m";
        __attribute__((unused)) static const char* dark_gray     = "\033[90m";
        __attribute__((unused)) static const char* light_red     = "\033[91m";
        __attribute__((unused)) static const char* light_green   = "\033[92m";
        __attribute__((unused)) static const char* light_yellow  = "\033[93m";
        __attribute__((unused)) static const char* light_blue    = "\033[94m";
        __attribute__((unused)) static const char* light_magenta = "\033[95m";
        __attribute__((unused)) static const char* light_cyan    = "\033[96m";
        __attribute__((unused)) static const char* white         = "\033[97m";
    } /* fg */
    //! Background Coloring
    namespace bg {
        //! Resets the background to default colors
        __attribute__((unused)) static const char* def           = "\033[49m"; // Resets the background to default colors
        __attribute__((unused)) static const char* black         = "\033[40m";
        __attribute__((unused)) static const char* red           = "\033[41m";
        __attribute__((unused)) static const char* green         = "\033[42m";
        __attribute__((unused)) static const char* yellow        = "\033[43m";
        __attribute__((unused)) static const char* blue          = "\033[44m";
        __attribute__((unused)) static const char* magenta       = "\033[45m";
        __attribute__((unused)) static const char* cyan          = "\033[46m";
        __attribute__((unused)) static const char* light_gray    = "\033[47m";
        __attribute__((unused)) static const char* dark_gray     = "\033[100m";
        __attribute__((unused)) static const char* light_red     = "\033[101m";
        __attribute__((unused)) static const char* light_green   = "\033[102m";
        __attribute__((unused)) static const char* light_yellow  = "\033[103m";
        __attribute__((unused)) static const char* light_blue    = "\033[104m";
        __attribute__((unused)) static const char* light_magenta = "\033[105m";
        __attribute__((unused)) static const char* light_cyan    = "\033[106m";
        __attribute__((unused)) static const char* white         = "\033[107m";
    } /* bg */
    __attribute__((unused)) static const char* reset       = "\033[0m";
    __attribute__((unused)) static const char* bold        = "\033[1m";
    __attribute__((unused)) static const char* unbold      = "\033[21m";
    __attribute__((unused)) static const char* dim         = "\033[1m";
    __attribute__((unused)) static const char* undim       = "\033[22m";
    __attribute__((unused)) static const char* underline   = "\033[4m";
    __attribute__((unused)) static const char* ununderline = "\033[24m";
    __attribute__((unused)) static const char* blink       = "\033[5m";
    __attribute__((unused)) static const char* unblink     = "\033[25m";
    __attribute__((unused)) static const char* invert      = "\033[7m";
    __attribute__((unused)) static const char* uninvert    = "\033[27m";
    __attribute__((unused)) static const char* hide        = "\033[8m";
    __attribute__((unused)) static const char* unhide      = "\033[27m";
    __attribute__((unused)) static const char* clear       = "\033c";
} /* term */

namespace logging {

//! Different levels of severity for logs
enum class level : int {
    MAX = 5,
    DIRE = 4,
    FAIL = 3,
    WARN = 2,
    INFO = 1,
    NOTE = 0
};

//! Printable strings for each logging level
static const char* level_strings[5] = {
    "NOTE",
    "INFO",
    "WARN",
    "FAIL",
    "DIRE"
};

//! Coloring for different levels (level string color, message color)
static constexpr std::pair<const char*, const char*> level_colors[5] = {
    {"\033[36m", "\033[96m"}, // NOTE:  dark cyan, light cyan
    {"\033[37m", "\033[97m"}, // Info:  light gray, white
    {"\033[33m", "\033[93m"}, // Warn:  yellow, light yellow
    {"\033[31m", "\033[91m"}, // FAIL:  red, light red
    {"\033[35m", "\033[95m"}  // DIRE:  magenta, light magenta
};

//! A single Log to be passed between Conusmers
struct LogInfo {
    //! The time this log was generated
    std::chrono::time_point<std::chrono::high_resolution_clock> time;
    //! The severity level of this log
    logging::level level;
    //! The name of the channel this log is for (might be empty)
    std::string name;

    /**
     * @brief formats a message with the requisite info
     * @param[in] message 
     **/
    std::string format(const std::string& message, const bool& color) const {
        auto msg_time = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count())*1e-9;
        auto& level_string = level_strings[static_cast<int>(level)];
        if(color) {
            auto& level_color = level_colors[static_cast<int>(level)].first;
            auto& message_color = level_colors[static_cast<int>(level)].second;

            if(name == "")
                return string_format("%s[%s%s%s][%s%3.9f%s]: %s%s",
                    message_color, level_color, level_string, message_color,
                    level_color, msg_time, message_color, message.c_str(), term::fg::def
                );
            else
                return string_format("%s[%s%s%s][%s%3.9f%s][%s%s%s]: %s%s",
                    message_color, level_color, level_string, message_color,
                    level_color, msg_time, message_color,
                    level_color, name.c_str(), message_color,
                    message.c_str(), term::fg::def
                );
        }

        if(name == "")
            return string_format("[%s][%3.9f]: %s",
                level_string, msg_time, message.c_str()
            );
        else
            return string_format("[%s][%3.9f][%s]: %s",
                level_string, msg_time, name.c_str(), message.c_str()
            );
    }
};

//! Interface for consumers to consume
class Consumer {
public:
    /**
     * @brief logs an unnamed log
     * @param[in] info info about the message being processed
     * @param[in] message the  message to process
     **/
    virtual void log(const LogInfo& info, const std::string& message) = 0;
    /**
     * @brief sets the logging levels
     * @param[in] level_low the severity level below which logs will not be consumed
     * @param[in] level_high the severity level above which logs will not be consumed
     **/
    virtual void set_levels(level level_low, level level_high) = 0;
    /**
     * @brief sets the low logging level
     * @param[in] level_low the severity level below which logs will not be consumed
     **/
    virtual void set_level_low(level level_low) = 0;
    /**
     * @brief sets the high logging level
     * @param[in] level_high the severity level above which logs will not be consumed
     **/
    virtual void set_level_high(level level_high) = 0;
}; /* interface Consumer */

//! Consumes logs and prints them to stdout, see Consumer for member-function documentation
class StdoutConsumer : public Consumer {
public:
    typedef std::shared_ptr<StdoutConsumer> Ptr;

    static Ptr newPtr() {
        return std::make_shared<StdoutConsumer>();
    }

    /**
     * @brief initialize a logger that prints to stdout, default level is NOTE
     */
    StdoutConsumer():
        current_level_low_(level::INFO),
        current_level_high_(level::MAX)
    {};

    //! See Consumer::log
    void log(const LogInfo& info, const std::string& message) {
        if(info.level < current_level_low_ || info.level >= current_level_high_) return;

        std::cout << info.format(message, true) << std::endl;
    }

    //! See Consumer::set_levels
    void set_levels(level level_low, level level_high) {
        if(level_low < level::NOTE) level_low = level::NOTE;
        if(level_low > level::MAX) level_low = level::MAX;
        if(level_high < level::NOTE) level_high = level::NOTE;
        if(level_high > level::MAX) level_high = level::MAX;

        current_level_low_ = level_low;
        current_level_high_ = level_high;
    }

    //! See Consumer::set_level_low
    void set_level_low(level level_low) {
        if(level_low < level::NOTE) level_low = level::NOTE;
        if(level_low > level::MAX) level_low = level::MAX;

        current_level_low_ = level_low;
    }

    //! See Consumer::set_level_high
    void set_level_high(level level_high) {
        if(level_high < level::NOTE) level_high = level::NOTE;
        if(level_high > level::MAX) level_high = level::MAX;

        current_level_high_ = level_high;
    }

private:
    //! The current logging level (high pass)
    level current_level_low_;
    //! The current logging level (low pass)
    level current_level_high_;

}; /* class StdoutConsumer */

void t();

inline void f() {

}

//! Consumes logs and writes them to a log file, see Consumer for member-function documentation
class FileConsumer : public Consumer {
public:
    typedef std::shared_ptr<FileConsumer> Ptr;

    static Ptr newPtr(std::string file_name, bool num) {
        return std::make_shared<FileConsumer>(file_name, num);
    }

    /**
     * @brief initialize a logger that prints to stdout, default level is level::NOTE - level::DIRE
     */
    FileConsumer(std::string file_name, bool num = true):
        current_level_low_(level::NOTE),
        current_level_high_(level::DIRE),
        file_name_(file_name),
        num_(num),
        safe_(true)
    {
        if(num_) {
            int number = 0;
            bool found = false;
            std::string num_file_name;
            struct stat buffer; 
            while(number < 1000 && !found) {
                num_file_name = file_name + "_" + std::to_string(number) + ".txt";
                found = stat (num_file_name.c_str(), &buffer) != 0;
                ++number;
            }
            if(!found) {
                safe_ = false;
                std::cerr << term::fg::red << "FileConsumer find num failed " << file_name << term::fg::def << std::endl;
            } else {
                file_name_ = num_file_name;
            }
        } else {
            file_name_ += ".txt";
        }
        if(safe_) {
            file_.open(file_name_);
        }
    };

    //! see Consumer::log
    void log(const LogInfo& info, const std::string& message) {
        if(info.level < current_level_low_ || info.level >= current_level_high_ || !safe_) return;

        file_ << info.format(message, false) << std::endl;
    }

    //! see Consumer::set_levels
    void set_levels(level level_low, level level_high) {
        if(level_low < level::NOTE) level_low = level::NOTE;
        if(level_low > level::MAX) level_low = level::MAX;
        if(level_high < level::NOTE) level_high = level::NOTE;
        if(level_high > level::MAX) level_high = level::MAX;

        current_level_low_ = level_low;
        current_level_high_ = level_high;
    }

    //! see Consumer::set_level_low
    void set_level_low(level level_low) {
        if(level_low < level::NOTE) level_low = level::NOTE;
        if(level_low > level::MAX) level_low = level::MAX;

        current_level_low_ = level_low;
    }

    //! see Consumer::set_level_high
    void set_level_high(level level_high) {
        if(level_high < level::NOTE) level_high = level::NOTE;
        if(level_high > level::MAX) level_high = level::MAX;

        current_level_high_ = level_high;
    }

private:
    //! The current logging level (high pass)
    level current_level_low_;
    //! The current logging level (low pass)
    level current_level_high_;
    //! The path / name of the file to write to (no file type suffix)
    std::string file_name_;
    //! Whether or not to append number the logs 
    bool num_;
    //! Output stream!
    std::ofstream file_;
    //! Whether or not it is safe to write
    bool safe_;

}; /* class StdoutConsumer */

//! Internal variables not to be touched by end application
namespace _internal {
    //! Consumers for log messages
    static std::map<std::string, std::shared_ptr<Consumer>> consumers_;
    //! Whether or not logging is intialized
    static bool init_complete_ = false;

    /**
     * @brief initializes logging, just generates a stdout consumer
     **/
    inline void init_logging() {
        if(_internal::init_complete_) return;
        init_complete_ = true;

        auto std_logger = std::make_shared<StdoutConsumer>();
        consumers_["stdout"] = std_logger;
    }
} /* namespace _internal */

/**
 * @brief sets the logging levels for all applications
 * @param[in] level_low the level to log at (high pass)
 * @param[in] level_high the level to log at (low pass)
 **/
inline void set_levels(level level_low, level level_high) {
    _internal::init_logging();

    for(const auto& consumer : _internal::consumers_) {
        consumer.second->set_levels(level_low, level_high);
    }
}

/**
 * @brief sets the logging level for all applications
 * @param[in] level_low the level to log at (high pass)
 **/
inline void set_level_low(level level_low) {
    _internal::init_logging();

    for(const auto& consumer : _internal::consumers_) {
        consumer.second->set_level_low(level_low);
    }
}

/**
 * @brief sets the logging level for all applications
 * @param[in] level_low the level to log at (low pass)
 **/
inline void set_level_high(level level_high) {
    _internal::init_logging();

    for(const auto& consumer : _internal::consumers_) {
        consumer.second->set_level_high(level_high);
    }
}

/**
 * @brief gets a consumer by name
 * @param[in] name the name of the consumer to get
 * @return a pointer to the consumer (or nullptr)
 **/
inline std::shared_ptr<Consumer> get_consumer(const std::string& name) {
    _internal::init_logging();

    auto it = _internal::consumers_.find(name);
    if(it != _internal::consumers_.end()) {
        return it->second;
    }
    return nullptr;
}

/**
 * @brief add a consumer with a given name
 * @param[in] name the name to give the consumer
 * @param[in] consumer the pointer to the consumer to add
 * @return true if succesfully added, false if not (already exists)
 **/
inline bool add_consumer(const std::string& name, const std::shared_ptr<Consumer>& consumer) {
    _internal::init_logging();
    
    auto it = _internal::consumers_.find(name);
    if(it != _internal::consumers_.end()) {
        return false;
    }
    _internal::consumers_[name] = consumer;
    return true;
}

inline void delete_consumer(const std::string& name) {
    _internal::init_logging();
    _internal::consumers_.erase(name);
}

/**
 * @brief logs a message
 * @param[in] level the severity of the message
 * @param[in] message the message to log
 **/
inline void log(level level, const std::string& message) {
    _internal::init_logging();
    
    LogInfo nlog;
    nlog.time = std::chrono::high_resolution_clock::now();
    nlog.level = level;
    for(const auto& consumer : _internal::consumers_) {
        consumer.second->log(nlog, message);
    }
}

/**
 * @brief logs a message to a channel
 * @param[in] name the channel this log belongs to
 * @param[in] level the severity of the message
 * @param[in] message the message to log
 **/
inline void log(std::string name, level level, const std::string& message) {
    _internal::init_logging();

    LogInfo nlog;
    nlog.time = std::chrono::high_resolution_clock::now();
    nlog.name = name;
    nlog.level = level;
    for(const auto& consumer : _internal::consumers_) {
        consumer.second->log(nlog, message);
    }
}


/**
 * @brief logs a message with formatting
 * @param[in] name the channel this log belongs to
 * @param[in] level the severity of the message
 * @param[in] format the message to log
 * @param[in] args variadic args for formatting of the message
 **/
template<typename ... Args>
inline void log(level level, const std::string& format, Args ... args) {
    log(level, string_format(format, args ...));
}

/**
 * @brief logs a message to a channel with formatting
 * @param[in] level the severity of the message
 * @param[in] format the message to log
 * @param[in] args variadic args for formatting of the message
 **/
template<typename ... Args>
inline void log(std::string name, level level, const std::string& format, Args ... args) {
    log(name, level, string_format(format, args ...));
}

} /* namespace logging */
} /* namespace jutils */

#endif /* JUTILS_LOGGING_H_ */
