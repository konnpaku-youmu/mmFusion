#ifndef MMWAVE_H
#define MMWAVE_H

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include "utilities.h"

namespace mmfusion
{
    class Radar
    {
    private:
        mmfusion::SystemConf *_cfg;

        boost::asio::serial_port *_cmd_port_ptr;

        mmfusion::deviceStatus _status;

        void _display_properties();

        boost::system::error_code _write_to_serial(std::string &);

        boost::system::error_code _read_from_serial(std::string &);

    public:
        Radar(mmfusion::SystemConf &);

        ~Radar();

        void configure();

        void toggle();
    };

} // namespace mmfusion

#endif