#pragma once

#include <iostream>
#include <sstream>

#ifdef USE_GLOG
#define GLOG_USE_GLOG_EXPORT
#include <glog/logging.h>
#endif

#define BOPT_ENABLE_LOGGING
#ifdef BOPT_ENABLE_LOGGING

namespace rmp {

/**
 * @brief Basic logging class with the option to use either standard stream, or
 * google logging. Disable the logging with ENABLE_LOGGING
 *
 */
class Logger {
   public:
    enum class Level { INFO, WARNING, ERROR };

    class Stream {
       public:
        Stream(Level level) : level_(level) {}

        template <typename T>
        Stream& operator<<(const T& value) {
            buffer_ << value;
            return *this;
        }

        ~Stream() {
#ifdef USE_GLOG
            switch (level_) {
                case Level::INFO:
                    LOG(INFO) << buffer_.str();
                    break;
                case Level::WARNING:
                    LOG(WARNING) << buffer_.str();
                    break;
                case Level::ERROR:
                    LOG(ERROR) << buffer_.str();
                    break;
            }
#else
            std::ostream& out =
                (level_ == Level::ERROR) ? std::cerr : std::cout;
            out << toString(level_) << buffer_.str() << std::endl;
#endif
        }

       private:
        Level level_;
        std::ostringstream buffer_;

#ifndef USE_GLOG
        std::string toString(Level level) {
            switch (level) {
                case Level::INFO:
                    return "[INFO] ";
                case Level::WARNING:
                    return "[WARNING] ";
                case Level::ERROR:
                    return "[ERROR] ";
            }
            return "";
        }
#endif
    };

    static Stream info() { return Stream(Level::INFO); }
    static Stream warn() { return Stream(Level::WARNING); }
    static Stream error() { return Stream(Level::ERROR); }
};

#else
class Logger {
   public:
    class Stream {
       public:
        template <typename T>
        Stream& operator<<(const T& value) {
            return *this;
        }

        ~Stream() {}
    };
    static Stream info() { return Stream(); }
    static Stream warn() { return Stream(); }
    static Stream error() { return Stream(); }
};

#endif

}  // namespace rmp