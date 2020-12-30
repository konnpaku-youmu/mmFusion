#ifndef MMWAVE_H
#define MMWAVE_H

#include "utilities.h"

using namespace boost::asio::ip;

namespace mmfusion
{
    struct
    {
        uint32_t seq;
        uint64_t byte_cnt = 0;

        std::vector<uint16_t> raw_adc;

    } typedef RawDCAPacket;

    struct
    {
        size_t id;
        std::vector<RawDCAPacket> data;
    } typedef DataFrame;

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

    class DCA1000 : public MultiThreading, Device
    {
    private:
        boost::asio::io_service _io_srv;

        udp::socket *_socket;

        udp::endpoint _remote_ep;

        boost::array<char, 1466> _buf;

        size_t _frame_len;

        std::vector<DataFrame> _frame_list;

        void _make_packet(char **, char *, mmfusion::RawDCAPacket &);

        bool _frame_check(const DataFrame &);

        void _start_receive();

        void _handle_receive(const boost::system::error_code, size_t);

    protected:
        void entryPoint();

    public:
        DCA1000(mmfusion::SystemConf &, pthread_mutex_t &);

        ~DCA1000();

        void configure();
    };

} // namespace mmfusion

#endif