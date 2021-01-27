#ifndef LOGGER_H
#define LOGGER_H

#include "utilities.h"

namespace mmfusion
{
    class Logger
    {
    private:
        std::string _log_path;
    
    public:
        Logger(const std::string &);
        
        ~Logger();
    };
} // namespace mmfusion
#endif