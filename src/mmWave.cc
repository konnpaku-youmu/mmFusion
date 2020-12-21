#include "mmWave.h"

namespace mmfusion
{
    Radar::Radar(mmfusion::SystemConf &cfg, pthread_mutex_t &mut)
    {
        this->_cfg = &cfg;
        this->_status = mmfusion::deviceStatus::INIT;
        this->_mutex = mut;

        this->_display_properties();
    }

    Radar::~Radar()
    {
    }

    void Radar::_display_properties()
    {
        std::cout << "\033[1;34m";
        std::cout << "---" << std::endl;
        std::cout << "mmWave Radar properties" << std::endl;
        std::cout << "---" << std::endl;
        std::cout << "Radar model: " << this->_cfg->radar_model << std::endl;
        std::cout << "Command port: " << this->_cfg->cmd_port << std::endl;
        std::cout << "Baud rate: " << this->_cfg->baud_rate << std::endl;
        std::cout << "---" << std::endl;

        /* wait for confirmation before continue */
        std::cout << "Continue?(Y/n)";
        std::cout << "\033[0m";
        mmfusion::waitBeforeContinue();

        return;
    }

    void Radar::configure()
    {
        try
        {
            // instantiate & initialize serial port
            boost::asio::io_service io;
            this->_cmd_port_ptr = new boost::asio::serial_port(io);
            this->_cmd_port_ptr->open(this->_cfg->cmd_port);
        }
        catch (const std::exception &e)
        {
            std::cerr << "\033[1;31m";
            std::cerr << "Cannot open serial port: " << this->_cfg->cmd_port << std::endl;
            std::cerr << "\033[0m";
            std::cerr << e.what() << '\n';
        }

        /* set up options for UART communication */
        boost::asio::serial_port::baud_rate baud_rate(this->_cfg->baud_rate);
        boost::asio::serial_port::flow_control flow_ctl(boost::asio::serial_port::flow_control::none);
        boost::asio::serial_port::parity parity(boost::asio::serial_port::parity::none);
        boost::asio::serial_port::character_size c_size(8);

        this->_cmd_port_ptr->set_option(baud_rate);
        this->_cmd_port_ptr->set_option(flow_ctl);
        this->_cmd_port_ptr->set_option(parity);
        this->_cmd_port_ptr->set_option(c_size);

        /* send full profile configuration via serial port for initialization */
        for (auto cmd : this->_cfg->cmd_list)
        {
            // wait before sending sensorStart command to radar
            if (std::strcmp(cmd.c_str(), "sensorStart\r\n") == 0)
            {
                std::cout << "\033[1;34m";
                std::cout << "Ready to try \"sensorStart\". Continue?(Y/n)";
                std::cout << "\033[0m";
                mmfusion::waitBeforeContinue();
            }

            std::string response;
            assert(!this->_write_to_serial(cmd));
            assert(!this->_read_from_serial(response));
            std::cout << response << std::endl;

            usleep(100000);
        }

        this->_status = mmfusion::deviceStatus::CONFIGURED;

        return;
    }

    void Radar::entryPoint()
    {
        for(;;)
        {
            if(std::getchar() == '\n')
            {
                this->toggle();
            }
        }
    }

    void Radar::sensorStart()
    {
        std::string cmd = "sensorStart 0\r\n";
        std::string response;
        this->_write_to_serial(cmd);
        this->_read_from_serial(response);
        std::cout << response;
        
        if(response.find("Done"))
        {
            std::cout << "start" << std::endl;
        }

        return;
    }

    void Radar::sensorStop()
    {
        std::string cmd = "sensorStop\r\n";
        std::string response;
        this->_write_to_serial(cmd);
        this->_read_from_serial(response);
        std::cout << response;

        if(response.find("Done") == std::string::npos)
        {
            std::cout << "stop" << std::endl;
        }

        return;
    }

    void Radar::toggle()
    {
        std::string cmd;
        std::string response;
        switch (this->_status)
        {
        case mmfusion::deviceStatus::CONFIGURED:
            this->sensorStart();
            this->_status = RUNNING;
            break;
        case mmfusion::deviceStatus::RUNNING:
            cmd = "sensorStop\r\n";
            this->_write_to_serial(cmd);
            this->_status = CONFIGURED;
            this->_read_from_serial(response);
            std::cout << response;
            break;
        default:
            break;
        }

        return;
    }

    boost::system::error_code Radar::_write_to_serial(std::string &message)
    {
        boost::system::error_code ec;
        boost::asio::write(*this->_cmd_port_ptr, boost::asio::buffer(message), ec);
        return ec;
    }

    boost::system::error_code Radar::_read_from_serial(std::string &message)
    {
        message = "";
        size_t bytes = 0;
        boost::asio::streambuf buf;
        boost::system::error_code ec;

        /* Synchronous reading*/
        while (message.find("\r\n") == std::string::npos)
        {
            bytes += boost::asio::read(*this->_cmd_port_ptr, buf, boost::asio::transfer_exactly(1), ec);
            if (ec)
            {
                std::cerr << "Reading UART response failed..." << std::endl;
                break;
            }

            boost::asio::streambuf::const_buffers_type cont_buf = buf.data();
            message = std::string(boost::asio::buffers_begin(cont_buf), boost::asio::buffers_begin(cont_buf) + bytes);
        }

        return ec;
    }

    DCA1000::DCA1000(mmfusion::SystemConf &cfg)
    {
        this->_cfg = &cfg;
        this->_status = mmfusion::deviceStatus::INIT;

        return;
    }

    DCA1000::~DCA1000()
    {
    }

    void DCA1000::configure()
    {
        this->_io = new boost::asio::io_service();
        this->_socket_ptr = new boost::asio::ip::udp::socket(*this->_io);

        return;
    }

    void DCA1000::readRawADC()
    {
        this->_socket_ptr->open(boost::asio::ip::udp::v4());
        this->_socket_ptr->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(this->_cfg->dca_addr),
                                                               this->_cfg->dca_data_port));

        this->_wait();

        this->_io->run();
    }

    void DCA1000::_handle_recv(const boost::system::error_code &ec, size_t bytes_transferred)
    {
        if (ec)
        {
            std::cerr << "Read UDP packet failed" << std::endl;
        }

        std::cout << "Received: " << std::string(this->_recv_buf.begin(), this->_recv_buf.begin() + bytes_transferred) << std::endl;
    }

    void DCA1000::_wait()
    {
        this->_socket_ptr->async_receive_from(boost::asio::buffer(this->_recv_buf),
                                              this->_remote_endpoint,
                                              boost::bind(&DCA1000::_handle_recv, this,
                                                          boost::asio::placeholders::error,
                                                          boost::asio::placeholders::bytes_transferred));
    }

} // namespace mmfusion