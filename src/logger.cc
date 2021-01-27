#include "logger.h"

namespace mmfusion
{
    Logger::Logger(const std::string &path)
    {
        this->_log_path = path;
    }

    Logger::~Logger()
    {

    }

}