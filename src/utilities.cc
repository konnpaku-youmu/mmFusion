#include "utilities.h"

namespace mmfusion
{
    SystemConf::SystemConf(const std::string &cfg_path)
    {
        try
        {
            cv::FileStorage cfg(cfg_path, cv::FileStorage::READ);
            std::string device_sys, device_calib;

            this->camera_pitch = this->camera_pitch * (M_PI / 180);

            // read from calibration result
            cfg["Camera"]["calib"]["savePath"] >> this->calib_conf_path;
            assert(!calib_conf_path.empty());
            cv::FileStorage calib_res = cv::FileStorage(this->calib_conf_path,
                                                        cv::FileStorage::READ);
            calib_res["device"] >> device_calib;
            calib_res["camera_matrix"] >> this->camera_mat;
            calib_res["dist_coeffs"] >> this->dist_coeffs;

            cfg["Camera"]["calib"]["dataSource"]["uri"] >> device_sys;
            if (std::strcmp(device_sys.c_str(), device_calib.c_str()) != 0)
            {
                this->device = "none";
            }
            else
            {
                this->device = device_sys;
            }

            cfg["Camera"]["install"]["height"] >> this->camera_height;
            cfg["Camera"]["install"]["pitch"] >> this->camera_pitch;
            this->camera_pitch *= (M_PI / 180);
            cfg["Camera"]["perceptionConfig"]["YOLOPath"]["cfg"] >> this->yolo_cfg;
            cfg["Camera"]["perceptionConfig"]["YOLOPath"]["weights"] >> this->yolo_weights;

            std::string coco_name_path;
            cfg["Camera"]["perceptionConfig"]["YOLOPath"]["names"] >> coco_name_path;
            std::ifstream name_file(coco_name_path);

            std::string name;
            while (std::getline(name_file, name))
            {
                this->coco_classes.push_back(name);
            }
            name_file.close();

            if (cfg["Camera"]["perceptionConfig"]["classFilter"]["enable"].real() == 1)
            {
                cv::FileNode _filter = cfg["Camera"]["perceptionConfig"]["classFilter"];
                for (cv::FileNodeIterator node = ++_filter.begin(); node != _filter.end(); ++node)
                {
                    std::string name = (*node).string();
                    for (size_t i = 0; i < this->coco_classes.size(); i++)
                    {
                        if (std::strcmp(name.c_str(), this->coco_classes[i].c_str()) == 0)
                        {
                            this->valid_classes.push_back(i);
                        }
                    }
                }
            }

            for (size_t i = 1; i <= this->coco_classes.size(); ++i)
            {
                float h, s, v;
                h = 2 * M_PI * ((float)i / this->coco_classes.size());
                s = 0.5 * M_1_PI * h;
                v = 0.5 * M_1_PI * h;
                cv::Mat3f hsv(cv::Vec3f(h, s, v));
                cv::Mat3f rgb;
                cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);

                cv::Scalar color(rgb.data[0], rgb.data[1], rgb.data[2]);
                this->class_colors.push_back(color);
            }

            /* load mmWave Radar parameters */
            cfg["mmWave"]["radar"]["commandPort"] >> this->cmd_port;
            cfg["mmWave"]["radar"]["baudRate"] >> this->baud_rate;

            std::string mmwave_profile;
            cfg["mmWave"]["radar"]["profileCfg"] >> mmwave_profile;

            if (mmwave_profile.find("6843"))
            {
                this->radar_model = "IWR6843";
            }
            else if (mmwave_profile.find("1642"))
            {
                this->radar_model = "IWR1642";
            }
            else
            {
                this->radar_model = "UNDEFINED";
            }

            std::ifstream profile_config(mmwave_profile);

            std::string command;
            while (std::getline(profile_config, command))
            {
                if (command.front() == '%')
                {
                    continue;
                }

                std::stringstream cmd;
                cmd << command << "\r\n";
                this->cmd_list.push_back(cmd.str());
            }

            // add sensorStop as final command
            this->cmd_list.push_back("sensorStop\r\n");
            profile_config.close();

            /* load DCA1000 configuration */
            cfg["mmWave"]["DCA1000"]["ip"] >> this->dca_addr;
            cfg["mmWave"]["DCA1000"]["cmdPort"] >> this->dca_cmd_port;
            cfg["mmWave"]["DCA1000"]["dataPort"] >> this->dca_data_port;
            cfg["mmWave"]["DCA1000"]["triggerMode"] >> this->trigger_mode;
            if (std::strcmp(this->trigger_mode.c_str(), "Software"))
            {
                cfg["mmWave"]["DCA1000"]["cfgPath"] >> this->dca_cfg_path;
            }

            cfg.release();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    bool MultiThreading::startThread(pthread_attr_t &attr)
    {
        return (pthread_create(&(this->_thread), &attr, _internal_thread_entry, this) == 0);
    }

    void MultiThreading::stopThread()
    {
        this->flag = 1;
        return (void)pthread_join(this->_thread, NULL);
    }

    void *MultiThreading::_internal_thread_entry(void *This)
    {
        pthread_mutex_lock(&(((MultiThreading *)This)->_mutex));
        ((MultiThreading *)This)->entryPoint();
        pthread_mutex_unlock(&(((MultiThreading *)This)->_mutex));

        return NULL;
    }

    int argmax(cv::Mat1d list)
    {
        float max = -1;
        int idx = -1;
#pragma omp parallel
#pragma omp for
        for (size_t col = 0; col < list.cols; ++col)
        {
            float curr_val = list.at<double>(col);
            if (curr_val > max)
            {
                max = curr_val;
                idx = col;
            }
        }
        return idx;
    }

    void padding(cv::Mat &src, cv::Mat &dst)
    {
        cv::Mat src_cp = src.clone();
        int w = src_cp.size().width;
        int h = src_cp.size().height;
        if (w > h)
        {
            dst = cv::Mat::zeros(cv::Size(w, w), src_cp.type());
            cv::Point2i tl(0, (int)((w - h) / 2));
            src_cp.copyTo(dst(cv::Rect(tl, src_cp.size())));
        }
        else if (w < h)
        {
            dst = cv::Mat::zeros(cv::Size(h, h), src_cp.type());
            cv::Point2i tl((int)((h - w) / 2), 0);
            src_cp.copyTo(dst(cv::Rect(tl, src_cp.size())));
        }
        else
        {
            dst = src;
        }

        return;
    }

    void waitBeforeContinue()
    {
        char yn;
        yn = std::getchar();

        std::cerr << "\033[1;31m";
        switch (yn)
        {
        case '\n':
            break;
        case 'Y':
            break;
        case 'y':
            break;
        case 'n':
            std::cerr << "Abort!" << std::endl;
            raise(SIGINT);
            break;
        default:
            std::cerr << "Invalid input. Abort!" << std::endl;
            raise(SIGINT);
            break;
        }
        std::cerr << "\033[0m";

        return;
    }

    std::vector<std::string> split(std::string &input, const std::string &delimiter)
    {
        std::string src = input;
        std::vector<std::string> res;

        while (!src.empty())
        {
            size_t pos = src.find(delimiter);
            if (pos != std::string::npos)
            {
                res.push_back(src.substr(0, pos));
                src.erase(0, pos + delimiter.size());
            }
            else
            {
                res.push_back(src);
                break;
            }
        }

        return res;
    }

    void getNormMat(Eigen::MatrixXcd &src, Eigen::MatrixXd &dst)
    {
        dst = Eigen::MatrixXd::Zero(src.rows(), src.cols());
        size_t row;
#pragma omp parallel for private(row)
        for (row = 0; row < src.rows(); ++row)
        {
            size_t col;
#pragma omp parallel for private(col)
            for (col = 0; col < src.cols(); ++col)
            {
                dst(row, col) = sqrt(src(row, col).real() * src(row, col).real() +
                                     src(row, col).imag() * src(row, col).imag());
            }
        }
        return;
    }

    Eigen::VectorXd cfarConv(Eigen::VectorXd &src, int window_size,
                             int stride, double threshold)
    {
        assert(window_size > 3 && window_size % 2 == 1);
        Eigen::VectorXd res = Eigen::VectorXd::Zero(src.rows());
        Eigen::VectorXd kernel = Eigen::VectorXd::Zero(window_size);

        // construct conv kernel
        int valid_cell = window_size / 4;
        for (int i = 0; i < valid_cell; ++i)
        {
            kernel(window_size - i - 1) = kernel(i) = -threshold * (0.5 / valid_cell);
        }
        kernel(window_size / 2) = 1;

        int i;
#pragma omp parallel num_threads(16)
#pragma omp for
        for (i = window_size / 2; i < src.rows() - (window_size / 2); i += stride)
        {
            res(i) = src.segment(i - (window_size / 2), window_size).dot(kernel);
        }

        return res;
    }

    void blur2D(Eigen::MatrixXd &src, Eigen::MatrixXd &dst, int radius)
    {
        dst = Eigen::MatrixXd::Zero(src.rows(), src.cols());
        assert(radius > 2 && radius % 2 == 1);
        Eigen::MatrixXd conv_kernel = Eigen::MatrixXd::Ones(radius, radius);
        conv_kernel /= radius * radius;

        Eigen::MatrixXd src_no_border = src.block(1, 1, src.rows() - 2, src.cols() - 2);
        Eigen::MatrixXd dst_no_border = Eigen::MatrixXd::Zero(src_no_border.rows(), src_no_border.cols());
        mmfusion::Conv2D(src_no_border, dst_no_border, conv_kernel);
        dst.block(1, 1, src.rows() - 2, src.cols() - 2) = dst_no_border;
        return;
    }

    void Conv2D(Eigen::MatrixXd &src, Eigen::MatrixXd &dst, Eigen::MatrixXd &kernel)
    {
        dst = Eigen::MatrixXd::Zero(src.rows(), src.cols());
        int row, col;
#pragma omp parallel for private(row, col)
        for (row = kernel.rows() / 2; row < src.rows() - (kernel.rows() / 2); ++row)
        {
            for (col = kernel.cols() / 2; col < src.cols() - (kernel.cols() / 2); ++col)
            {
                dst(row, col) = (src.block(row - kernel.rows() / 2, col - kernel.cols() / 2,
                                           kernel.rows(), kernel.cols())
                                     .array() *
                                 kernel.array())
                                    .sum();
            }
        }
    }
} // namespace mmfusion
