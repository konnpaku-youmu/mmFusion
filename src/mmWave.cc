#include "mmWave.h"

namespace mmfusion
{
    Radar::Radar(mmfusion::SystemConf &cfg)
    {
        this->_cfg = &cfg;

        try
        {
            // instantiate & initialize serial port
            boost::asio::io_service io;
            this->_cmd_port_ptr = new boost::asio::serial_port(io);
            this->_cmd_port_ptr->open(this->_cfg->cmd_port);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        // options for UART communication
        boost::asio::serial_port::baud_rate baud_rate(this->_cfg->baud_rate);
        boost::asio::serial_port::flow_control flow_ctl(boost::asio::serial_port::flow_control::none);
        boost::asio::serial_port::parity parity(boost::asio::serial_port::parity::none);
        boost::asio::serial_port::character_size c_size(8);

        this->_cmd_port_ptr->set_option(baud_rate);
        this->_cmd_port_ptr->set_option(flow_ctl);
        this->_cmd_port_ptr->set_option(parity);
        this->_cmd_port_ptr->set_option(c_size);

        // send full profile configuration via serial port for initialization
        for (auto cmd : this->_cfg->cmd_list)
        {
            // std::cout << cmd;
            boost::asio::write(*this->_cmd_port_ptr, boost::asio::buffer(cmd));

            std::string message = "";
            size_t bytes = 0;
            boost::asio::streambuf buf;
            while (message.find("\r\n") == std::string::npos)
            {
                boost::system::error_code ec;
                bytes += boost::asio::read(*this->_cmd_port_ptr, buf, boost::asio::transfer_exactly(1), ec);
                if (ec)
                {
                    break;
                }
                boost::asio::streambuf::const_buffers_type cont_buf = buf.data();
                message = std::string(boost::asio::buffers_begin(cont_buf), boost::asio::buffers_begin(cont_buf) + bytes);
            }
            std::cout << message << std::endl;

            usleep(50000);
        }
    }

    Radar::~Radar()
    {
    }

    void Radar::configure()
    {
    }
} // namespace mmfusion