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

    public:
        Radar(mmfusion::SystemConf &);

        ~Radar();

        void configure();
    };

} // namespace mmfusion

#endif