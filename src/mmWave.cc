#include "mmWave.h"

namespace mmfusion
{
    Radar::Radar(mmfusion::SystemConf &cfg, pthread_mutex_t &mut)
    {
        this->_cfg = &cfg;
        this->_status = mmfusion::deviceStatus::INIT;
        this->_mutex = mut;

        this->_display_properties();

        try
        {
            // instantiate & initialize serial port
            boost::asio::io_service io;
            this->_cmd_port_ptr = new boost::asio::serial_port(io);
            this->_cmd_port_ptr->open(this->_cfg->radar_cmd_port);

            /* set up options for UART communication */
            boost::asio::serial_port::baud_rate baud_rate(this->_cfg->baud_rate);
            boost::asio::serial_port::flow_control flow_ctl(boost::asio::serial_port::flow_control::none);
            boost::asio::serial_port::parity parity(boost::asio::serial_port::parity::none);
            boost::asio::serial_port::character_size c_size(8);

            this->_cmd_port_ptr->set_option(baud_rate);
            this->_cmd_port_ptr->set_option(flow_ctl);
            this->_cmd_port_ptr->set_option(parity);
            this->_cmd_port_ptr->set_option(c_size);
        }
        catch (const std::exception &e)
        {
            std::cerr << "\033[1;31m";
            std::cerr << "Cannot open serial port: " << this->_cfg->radar_cmd_port << std::endl;
            std::cerr << "\033[0m";
            std::cerr << e.what() << '\n';
        }

        return;
    }

    Radar::~Radar()
    {
        delete this->_cmd_port_ptr;
        delete this->_data_port_ptr;
    }

    /**
     * @brief A method to display mmWave Radar related information, including radar model,
     *        serial port for sending commands and its baud rate. This method will return
     *        after some keyboard input from user
     * 
     */
    void Radar::_display_properties()
    {
        std::cout << "\033[0;92m";
        std::cout << "---" << std::endl;
        std::cout << "mmWave Radar properties" << std::endl;
        std::cout << "---" << std::endl;
        std::cout << "Radar model: " << this->_cfg->radar_model << std::endl;
        std::cout << "Command port: " << this->_cfg->radar_cmd_port << std::endl;
        std::cout << "Baud rate: " << this->_cfg->baud_rate << std::endl;
        std::cout << "---" << std::endl;

        /* wait for confirmation before continue */
        std::cout << "Continue?(Y/n)";
        std::cout << "\033[0m";
        mmfusion::waitBeforeContinue();

        return;
    }
    
    /**
     * @brief send full profile configuration to radar via serial port
     * 
     */
    void Radar::configure()
    {
        for (auto cmd : this->_cfg->radar_cmd_list)
        {
            // std::cout << cmd << std::endl;
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

            usleep(100000);
        }

        this->_status = mmfusion::deviceStatus::CONFIGURED;

        return;
    }

    void Radar::entryPoint()
    {
        while (!flag)
            ;
        return;
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
            std::cout << "[INFO] Sensor is running. Press 'Enter' to pause..." << std::endl;
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
            std::cout << "[INFO] Sensor stopped. Press 'Enter' to resume..." << std::endl;
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

    /**
     * @brief A method to read serial data in a synchronous manner
     * 
     * @param message container to hold the serial data as characters
     * @return boost::system::error_code 
     */
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

    /**
     * @brief Construct a new DCA1000::DCA1000 object
     * 
     * @param cfg system configuration object
     * @param mut mutex lock
     */
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
        if (this->_cfg->capture_trigger_mode == 1)
        {
            this->configure();
        }

        /* Determine frame length */
        std::vector<std::string> tokens;
        int chirps_per_frame;

        for (auto cmd : this->_cfg->radar_cmd_list)
        {
            // get the number of Tx and Rx
            if (cmd.find("channelCfg") != std::string::npos)
            {
                tokens = split(cmd, " ");
                int rx_code = std::stoi(tokens[1]);
                int tx_code = std::stoi(tokens[2]);

                // The number of the set bits is equal to the number of enabled Rx antennas.
                tx_num = 0;
                rx_num = 0;
                while (rx_code != 0)
                {
                    rx_num += rx_code & 1;
                    rx_code >>= 1;
                }

                while (tx_code != 0)
                {
                    tx_num += tx_code & 1;
                    tx_code >>= 1;
                }
                this->_cfg->tx_num = tx_num;
                this->_cfg->rx_num = rx_num;
            }
            // get ADC samples per chirp
            else if (cmd.find("profileCfg") != std::string::npos)
            {
                tokens = split(cmd, " ");
                adc_samples = std::stoi(tokens[10]);
                this->_cfg->adc_samples = this->adc_samples;
            }
            // get number of chirps in one single frame
            else if (cmd.find("frameCfg") != std::string::npos)
            {
                tokens = split(cmd, " ");
                chirps_per_loop = std::stoi(tokens[2]) - std::stoi(tokens[1]) + 1;
                loops = std::stoi(tokens[3]);
                this->_cfg->chirp_loops = this->loops;
                chirps_per_frame = loops * chirps_per_loop;
            }
        }

        /* calculate frame length */
        /* (IQ) * (16bits) * (number of Tx) * (number of Rx) * (ADC samples per chirp) * (chirps in a frame)*/
        this->_frame_len = 2 * 2 * tx_num * rx_num * adc_samples * chirps_per_frame;
        std::cout << "Tx: " << this->tx_num << std::endl;
        std::cout << "Rx: " << this->rx_num << std::endl;
        std::cout << "ADC Samples: " << this->adc_samples << std::endl;
        std::cout << "Loops: " << loops << std::endl;
        std::cout << "Chirps per loop: " << chirps_per_loop << std::endl;
        std::cout << "[INFO] Frame length in bytes: " << this->_frame_len << std::endl;

        // initializing data container
        this->_raw_data.rw_lock = mmfusion::RWStatus::WRITING;
        this->_raw_data.data_flattened = Eigen::MatrixXcf::Zero(this->adc_samples,
                                                                this->rx_num * this->chirps_per_loop * this->loops);
        this->_raw_data.rw_lock = mmfusion::RWStatus::UNAVAILABLE;

        // Queue the async receive task
        this->_start_receive();

        this->_status = mmfusion::deviceStatus::CONFIGURED;

        return;
    }

    DCA1000::~DCA1000()
    {
        delete this->_socket;
    }

    void DCA1000::entryPoint()
    {
        this->_io_srv.run();
        return;
    }

    void DCA1000::_organize()
    {
        if (this->_raw_data.rw_lock == READING)
        {
            return;
        }

        this->_raw_data.rw_lock = mmfusion::RWStatus::WRITING;

        Eigen::VectorXi raw_adc(this->_frame_len >> 1);
        DataFrame *active_frame = &this->_frame_list.back();

        // serialize raw ADC data frame
        size_t num_count = 0;
        // #pragma omp parallel for private(num_count)
        for (auto packet : active_frame->data)
        {
            for (auto data : packet.raw_adc)
            {
                raw_adc(num_count) = data;
                num_count++;
            }
        }

        Eigen::Map<Eigen::MatrixXi> organized(raw_adc.data(),
                                              this->rx_num * this->tx_num * this->adc_samples * 2,
                                              this->loops * this->chirps_per_loop);

        Eigen::MatrixXcf raw_temp = Eigen::MatrixXcf::Zero(this->_raw_data.data_flattened.rows(),
                                                           this->_raw_data.data_flattened.cols());

        /* Old documentation is correct */
        int chirp;
#pragma omp parallel for private(chirp)
        for (chirp = 0; chirp < organized.cols(); ++chirp)
        {
            Eigen::Map<Eigen::MatrixXi> one_chirp(organized.col(chirp).data(),
                                                  this->adc_samples * 2,
                                                  this->rx_num * this->tx_num);

            Eigen::MatrixXcf cplx_raw(this->rx_num * this->tx_num,
                                      this->adc_samples);
            int rx;
#pragma omp parallel for private(rx)
            for (rx = 0; rx < this->rx_num * this->tx_num; ++rx)
            {
                Eigen::Map<Eigen::MatrixXi> one_rx(one_chirp.col(rx).data(),
                                                   2, this->adc_samples);
                int sample;
#pragma omp parallel sections
                {
#pragma omp section
#pragma omp parallel for private(sample)
                    for (sample = 0; sample < this->adc_samples; sample += 2)
                    {
                        cplx_raw(rx, sample) = Eigen::scomplex((int16_t)one_rx(0, sample + 1) * LSB,
                                                               (int16_t)one_rx(0, sample) * LSB);
                    }
#pragma omp section
#pragma omp parallel for private(sample)
                    for (sample = 1; sample < this->adc_samples; sample += 2)
                    {
                        cplx_raw(rx, sample) = Eigen::scomplex((int16_t)one_rx(1, sample) * LSB,
                                                               (int16_t)one_rx(1, sample - 1) * LSB);
                    }
                }
            }
            raw_temp.block(0, chirp * this->rx_num * this->tx_num,
                           cplx_raw.cols(), cplx_raw.rows()) = cplx_raw.transpose();
        }

        /* re-arrange by Rx */
        int rx;
#pragma omp parallel for private(rx)
        for (rx = 0; rx < this->rx_num * this->tx_num; ++rx)
        {
            Eigen::MatrixXcf rx_n = Eigen::MatrixXcf::Map(raw_temp.data() + (rx * raw_temp.rows()),
                                                          raw_temp.rows(), raw_temp.cols() / (this->rx_num * tx_num),
                                                          Eigen::OuterStride<>(
                                                              this->rx_num * this->tx_num * raw_temp.rows()));
            this->_raw_data.data_flattened.block(0, rx * rx_n.cols(),
                                                 rx_n.rows(), rx_n.cols()) = rx_n;
        }

        this->_raw_data.rw_lock = mmfusion::RWStatus::AVAILABLE;

        return;
    }

    mmfusion::deviceStatus DCA1000::getStatus()
    {
        return this->_status;
    }

    bool DCA1000::getRawData(Eigen::MatrixXcf &data)
    {
        bool ret = false;

        if (this->_raw_data.rw_lock == ACCESSED)
        {
            ret = false;
        }
        else if (this->_raw_data.rw_lock == AVAILABLE &&
                 this->_raw_data.data_flattened.rows() > 0 &&
                 this->_raw_data.data_flattened.cols() > 0)
        {
            this->_raw_data.rw_lock = mmfusion::RWStatus::READING;
            data = this->_raw_data.data_flattened;
            this->_raw_data.rw_lock = mmfusion::RWStatus::ACCESSED;
            ret = true;
        }

        return ret;
    }

    void DCA1000::_start_receive()
    {
        if (flag)
        {
            // if Multithread::flag is raised, stop entryPoint() thus join the thread
            this->_io_srv.stop();
        }

        this->_status = mmfusion::deviceStatus::CONFIGURED;
        // limit the size of frame collection to 5 frames
        if (this->_frame_list.size() > 5)
        {
            _frame_list.pop_front();
        }

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
                // If previous data frames exist, it means that the current data packet /
                // is the last chunk of data of the lastest dataframe in the data frame collection. /
                // We pack the corresponding data whose length will be less than 1456 Bytes /
                // meanwhile the pointer is moved to the start of the new data

                DataFrame *last_frame = &this->_frame_list.back();
                char *end_position = phead + curr_frame_end;
                this->_make_packet(&phead, end_position, packet);
                last_frame->data.push_back(packet);

                // Since this packet contains data belongs to two consecutive dataframes, /
                // after we finished constructing the old dataframe, we should clear the /
                // ADC data in the current packet(which belongs to the old data frame)
                // to hold data of the new dataframe, which will reuse the packet instance.
                packet.raw_adc.clear();
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
                // Check data integrity. If failed, abandon the last frame while the /
                // incrementation of frame id is unaffected.
                new_frame.id = this->_frame_list.back().id + 1;

                if (this->_frame_check(this->_frame_list.back()))
                {
                    this->_organize();
                }
                else
                {
                    std::cout << "\nFrame: " << this->_frame_list.back().id << " is probably malformed" << std::endl;
                    this->_frame_list.pop_back();
                }
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

        this->_status = mmfusion::deviceStatus::RUNNING;

        // trigger receive thread again
        this->_start_receive();

        return;
    }

    inline void DCA1000::_make_packet(char **begin, char *end, mmfusion::RawDCAPacket &packet)
    {
        size_t byte_len = end - *begin;
        size_t vector_len = byte_len / sizeof(uint16_t);
        packet.raw_adc = std::vector<uint16_t>(vector_len);
        uint16_t *raw_data = packet.raw_adc.data();
        std::memcpy(raw_data, *begin, byte_len);
        *begin += byte_len;
        return;
    }

    inline bool DCA1000::_frame_check(const DataFrame &frame)
    {
        size_t total_len = 0;

        int i;
        for (i = 0; i < frame.data.size(); ++i)
        {
            total_len += (frame.data[i].raw_adc.size() << 1);
        }

        return (total_len == this->_frame_len);
    }

    void DCA1000::configure()
    {
        return;
    }
} // namespace mmfusion