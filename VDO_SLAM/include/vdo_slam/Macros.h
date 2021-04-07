#include <iostream>
#include <chrono>
#include <ctime>
#include <string>

#pragma once



#ifdef VDO_DEBUG
#define USE_VDO_DEBUG
#endif

#define noop


#define EXTRACT_FILE(file_path) \
    file_path.substr(file_path.rfind("/") + 1)

#define LOGGING_STREAM_FORMATTER(level, x) \
    "[ VDO: " << level << "] [" << EXTRACT_FILE(std::string(__FILE__)) << ": " << __LINE__ << "] " << x

#ifdef USE_VDO_DEBUG
    #define VDO_DEBUG_MSG(x) \
        std::cout << LOGGING_STREAM_FORMATTER("DEBUG", x) << std::endl;

#else
    #define VDO_DEBUG_MSG(x) noop
#endif

#define VDO_INFO_MSG(x) \
    std::cout << LOGGING_STREAM_FORMATTER("INFO", x) << std::endl;

#define VDO_WARN_MSG(x) \
    std::cout << LOGGING_STREAM_FORMATTER("WARN", x) << std::endl;

#define VDO_ERROR_MSG(x) \
    std::cout << LOGGING_STREAM_FORMATTER("ERROR", x) << std::endl;

#define VDO_CRITICAL_MSG(x) \
    std::cout << LOGGING_STREAM_FORMATTER("CRITICAL", x) << std::endl;
