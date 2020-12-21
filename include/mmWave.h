#ifndef MMWAVE_H
#define MMWAVE_H

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include "utilities.h"

namespace mmfusion
{
    class Radar : public MultiThreading, Device
    {
    private:
        boost::asio::serial_port *_cmd_port_ptr;

        void _display_properties();

        boost::system::error_code _write_to_serial(std::string &);

        boost::system::error_code _read_from_serial(std::string &);
    
    protected:
        void entryPoint();

    public:
        Radar(mmfusion::SystemConf &, pthread_mutex_t &);

        ~Radar();

        void configure();

        void sensorStart();

        void sensorStop();

        void toggle();
    };

    class DCA1000 : public Device
    {
    private:
        boost::asio::io_service *_io;

        boost::asio::ip::udp::socket *_socket_ptr;

        boost::asio::ip::udp::endpoint _remote_endpoint;

        boost::array<char, 65536> _recv_buf;

        void _wait();

        void _handle_recv(const boost::system::error_code &, size_t);

    public:
        DCA1000(mmfusion::SystemConf &);

        ~DCA1000();

        void readRawADC();

        void configure();
    };

} // namespace mmfusion

#endif