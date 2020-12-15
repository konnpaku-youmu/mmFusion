#include "mmWave.h"

namespace mmfusion
{
    Radar::Radar(mmfusion::SystemConf &cfg)
    {
        this->_cfg = &cfg;

        // instantiate & initialize serial port

        this->_cmd_port_ptr = new boost::asio::serial_port(this->_io);

        boost::asio::serial_port_base::baud_rate baud_rate(115200);
        this->_cmd_port_ptr->open(this->_cfg->cmd_port);
        this->_cmd_port_ptr->set_option(baud_rate);

        // send full profile configuration via serial port for initialization
        for (auto cmd : this->_cfg->cmd_list)
        {
            std::cout << cmd << std::endl;
            boost::asio::write(*this->_cmd_port_ptr, boost::asio::buffer(cmd));

            char data[100];

            boost::asio::async_read(*this->_cmd_port_ptr, boost::asio::buffer(data), boost::bind(&Radar::_async_serial_handle, this, data, _1, _2));

            usleep(100000);
        }
    }

    void Radar::_async_serial_handle(char buf[], boost::system::error_code ec,
                                     size_t bytes_transferred)
    {
        std::cout.write(buf, bytes_transferred);
    }

    Radar::~Radar()
    {
    }

    void Radar::configure()
    {
    }
} // namespace mmfusion