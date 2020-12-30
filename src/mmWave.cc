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

        std::cout << "[INFO] UDP socket is created..." << std::endl;

        /* write configuration to DCA1000 if using software config (SW2.5 -> SW_CONFIG) */
        if (std::strcmp(this->_cfg->trigger_mode.c_str(), "Software") == 0)
        {
            this->configure();
        }

        /* Determine frame length */
        int tx_num = 0, rx_num = 0, adc_samples = 0, chirps_per_frame = 0;
        std::vector<std::string> tokens;

        for (auto cmd : this->_cfg->cmd_list)
        {
            // get the number of Tx and Rx
            if (cmd.find("channelCfg") != std::string::npos)
            {
                tokens = split(cmd, " ");
                int rx_code = std::stoi(tokens[1]);
                int tx_code = std::stoi(tokens[2]);

                // The number of the set bits is equal to the number of enabled Rx antennas.
                while (rx_code != 0)
                {
                    rx_num += rx_code & 1;
                    rx_code >>= 1;
                }

                // The number of set bits in Tx config does not directly /
                // represent the number of equivalent Tx channels.
                switch (tx_code)
                {
                case 1: // Trivial case with Tx0 enabled
                    tx_num = 1;
                    break;
                case 4: // Trivial case with Tx2 enabled
                    tx_num = 1;
                    break;
                case 5: // Trivial case with both Tx0 and Tx2 enabled
                    tx_num = 2;
                    break;
                case 7: // Non-Trivial case when all three Tx antennas are enabled /
                    // Under this circumstance, antenna Tx1 expand the equivalent /
                    // Rx array with an additional row.
                    tx_num = 2;
                    break;
                default:
                    break;
                }
            }
            // get ADC samples per chirp
            else if (cmd.find("profileCfg") != std::string::npos)
            {
                tokens = split(cmd, " ");
                adc_samples = std::stoi(tokens[10]);
            }
            // get number of chirps in one single frame
            else if (cmd.find("frameCfg") != std::string::npos)
            {
                tokens = split(cmd, " ");
                int chirps_per_loop = std::stoi(tokens[2]) - std::stoi(tokens[1]) + 1;
                int loops = std::stoi(tokens[3]);
                chirps_per_frame = loops * chirps_per_loop;
            }
        }

        /* calculate frame length */
        /* (IQ) * (16bits) * (number of Tx) * (number of Rx) * (ADC samples per chirp) * (chirps in a frame)*/
        this->_frame_len = 2 * 2 * tx_num * rx_num * adc_samples * chirps_per_frame;
        std::cout << "[INFO] Frame length in bytes: " << this->_frame_len << std::endl;

        // Queue the async receive task
        this->_start_receive();

        this->_status = mmfusion::deviceStatus::CONFIGURED;

        return;
    }

    DCA1000::~DCA1000()
    {
    }

    void DCA1000::entryPoint()
    {
        this->_status = mmfusion::deviceStatus::RUNNING;
        this->_io_srv.run();

        return;
    }

    void DCA1000::_start_receive()
    {
        // receive UDP data and put them into this->_buf. /
        // Only RAW_MODE is allowed so that the length is maintained to 1456 bytes

        this->_socket->async_receive_from(boost::asio::buffer(this->_buf), this->_remote_ep,
                                          boost::bind(&DCA1000::_handle_receive, this,
                                                      boost::asio::placeholders::error,
                                                      boost::asio::placeholders::bytes_transferred));
        return;
    }

    void DCA1000::_handle_receive(const boost::system::error_code ec, size_t bytes_transferred)
    {
        /* Parsing Raw data */
        RawDCAPacket packet;
        char *phead = this->_buf.begin();

        // read first 4 bytes that represents the sequence number.
        std::memcpy(&packet.seq, phead, 4);
        phead += 4;

        // read the next 6 bytes that represent the bytes count of /
        // already sent raw ADC data. However, we copy 8 bytes in order to align /
        // with the size of uint64_t to prevent potential segmentation problems while /
        // immediately truncate the data to its original form by bitwise AND.
        std::memcpy(&packet.byte_cnt, phead, 8);
        packet.byte_cnt &= 0x00ffffffffffff;
        phead += 6;

        int curr_frame_end = this->_frame_len - (packet.byte_cnt % this->_frame_len);

        if (curr_frame_end == this->_frame_len)
        {
            // There will be circumstance when the start position of a data frame /
            // conincide with the start of a data packet, which will cause the /
            // curr_frame_end to be the _frame_len(which is typically much larger than 1456)
            curr_frame_end = 0;
        }

        if (curr_frame_end < 1456)
        {
            if (!this->_frame_list.empty())
            {
                // if previous data frames exist, it means that the current data packet /
                // is the last chunk of data of the lastest dataframe in the data frame collection.
                // We pack the corresponding data whose length will be less than 1456 Bytes.               this->_make_packet(phead, (&this->_buf[10] + curr_frame_end), packet);
                // meanwhile the pointer is moved to the start of the new data

                DataFrame *last_frame = &this->_frame_list.back();
                char *end_position = this->_buf.begin() + 10 + curr_frame_end;

                this->_make_packet(&phead, end_position, packet);
                last_frame->data.push_back(packet);
                if (!this->_frame_check(*last_frame))
                {
                    std::cout << "Frame: " << last_frame->id << " is probably malformed" << std::endl;
                }

                // Since this packet contains data belongs to two consecutive dataframes, /
                // after we finished constructing the old dataframe, we should clear the /
                // ADC data in the current packet(which belongs to the old data frame)
                // to hold data of the new dataframe, which will reuse the packet instance.
                packet.raw_adc.clear();

                // check data integrity
            }
            else
            {
                // if no previous data frame exists, we directly move the pointer to /
                // the start position of the incoming data frame.
                phead += curr_frame_end;
            }

            // Construct a new dataframe and push it into the data frame collection
            DataFrame new_frame;
            this->_make_packet(&phead, this->_buf.end(), packet);
            
            if (this->_frame_list.empty())
            {
                new_frame.id = 0;
            }
            else
            {
                new_frame.id = this->_frame_list.back().id + 1;
            }
            
            new_frame.data.push_back(packet);
            this->_frame_list.push_back(new_frame);
        }
        else
        {
            if (!this->_frame_list.empty())
            {
                this->_make_packet(&phead, this->_buf.end(), packet);
                this->_frame_list.back().data.push_back(packet);
            }
        }

        // trigger receive thread again
        this->_start_receive();

        return;
    }

    void DCA1000::_make_packet(char **begin, char *end, mmfusion::RawDCAPacket &packet)
    {
        while (*begin != end)
        {
            uint16_t raw_data;
            std::memcpy(&raw_data, *begin, 2);
            packet.raw_adc.push_back(raw_data);
            *begin += 2;
        }

        return;
    }

    bool DCA1000::_frame_check(const DataFrame &frame)
    {
        size_t seq_diff = 0;
        size_t byte_count_diff = 0;

        size_t total_packets = frame.data.size();
        size_t total_len = 0;

        for (auto packet : frame.data)
        {
            total_len += 2 * packet.raw_adc.size();
        }

        seq_diff = frame.data.back().seq - frame.data.front().seq + 1;
        byte_count_diff = frame.data.back().byte_cnt - frame.data.front().byte_cnt;

        return (total_len == this->_frame_len);
    }

    void DCA1000::configure()
    {
        return;
    }
} // namespace mmfusion