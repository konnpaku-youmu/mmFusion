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
        std::cout << "\033[0;92m";
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
            // wait before testing sensorStart command to radar
            if (std::strcmp(cmd.c_str(), "sensorStart\r\n") == 0)
            {
                std::cout << "\033[1;92m";
                std::cout << "Ready to try \"sensorStart\". Continue?(Y/n)";
                std::cout << "\033[0m";
                mmfusion::waitBeforeContinue();
            }

            std::string response;
            assert(!this->_write_to_serial(cmd));
            assert(!this->_read_from_serial(response));
            std::cout << response << std::endl;

            usleep(25000);
        }

        this->_status = mmfusion::deviceStatus::CONFIGURED;

        return;
    }

    void Radar::entryPoint()
    {
        for (;;)
        {
            if (std::getchar() == '\n')
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

        if (response.find("Done") != std::string::npos)
        {
            this->_status = mmfusion::deviceStatus::RUNNING;
            std::cout << "[INFO] Sensor is running. Press 'Enter' to pause...";
        }

        return;
    }

    void Radar::sensorStop()
    {
        std::string cmd = "sensorStop\r\n";
        std::string response;
        this->_write_to_serial(cmd);
        this->_read_from_serial(response);

        if (response.find("Done") != std::string::npos)
        {
            this->_status = mmfusion::deviceStatus::CONFIGURED;
            std::cout << "[INFO] Sensor stopped. Press 'Enter' to resume...";
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
            break;
        case mmfusion::deviceStatus::RUNNING:
            this->sensorStop();
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

    DCA1000::DCA1000(mmfusion::SystemConf &cfg, pthread_mutex_t &mut)
    {
        this->_cfg = &cfg;
        this->_status = mmfusion::deviceStatus::INIT;
        this->_mutex = mut;

        /* create UDP socket */
        this->_socket = new udp::socket(this->_io_srv,
                                        udp::endpoint(udp::v4(),
                                                      this->_cfg->dca_data_port));
        assert(this->_socket->is_open());

        std::cout << "[INFO]UDP socket is created..." << std::endl;

        /* write configuration to DCA1000 if using software config (SW2.5 -> SW_CONFIG) */
        if(std::strcmp(this->_cfg->trigger_mode.c_str(), "Software") == 0)
        {
            this->configure();
        }

        this->_start_receive();

        return;
    }

    DCA1000::~DCA1000()
    {
    }

    void DCA1000::entryPoint()
    {
        this->_io_srv.run();
        
        return;
    }

    void DCA1000::_start_receive()
    {
        this->_socket->async_receive_from(boost::asio::buffer(this->_buf), this->_remote_ep,
                                          boost::bind(&DCA1000::_handle_receive, this,
                                                      boost::asio::placeholders::error,
                                                      boost::asio::placeholders::bytes_transferred));
        return;
    }

    void DCA1000::_handle_receive(const boost::system::error_code ec, size_t bytes_transferred)
    {
        // for(auto byte:this->_buf)
        // {
        //     std::cout << std::setw(2) << std::setfill('0') << std::hex << (0xff & byte);
        // }
        // std::cout << std::endl << std::endl;

        /* Parsing Raw data */
        RawDCAPacket packet;
        char *phead = this->_buf.begin();

        std::memcpy(&packet.seq, phead, 4);
        phead += 4;

        std::memcpy(&packet.byte_cnt, phead, 8);
        packet.byte_cnt &= 0x00ffffffffffff;
        phead += 6;

        while(phead != this->_buf.end())
        {
            
        }

        std::cout << (packet.byte_cnt / (1 << 20)) << std::endl;

        this->_start_receive();
        
        return;
    }

    void DCA1000::configure()
    {
        return;
    }
} // namespace mmfusion