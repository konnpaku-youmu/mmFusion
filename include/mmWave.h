#ifndef MMWAVE_H
#define MMWAVE_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "utilities.h"

namespace mmfusion
{
    class Radar
    {
    private:
        mmfusion::SystemConf *_cfg;

        boost::asio::io_service _io;

        boost::asio::serial_port *_cmd_port_ptr;

        void _async_serial_handle(char *, boost::system::error_code, size_t);

    public:
        Radar(mmfusion::SystemConf &);

        ~Radar();

        void configure();
    };

} // namespace mmfusion

#endif