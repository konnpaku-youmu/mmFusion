#include "utilities.h"

namespace mmfusion
{
    SystemConf::SystemConf(const std::string &cfg_path)
    {
        try
        {
            cv::FileStorage system_conf(cfg_path, cv::FileStorage::READ);
            system_conf["Camera"]["enable"] >> enable_cam;
            if (enable_cam == 1)
            {
                /* Reading from camera calibration file */
                system_conf["Camera"]["calib"]["savePath"] >> this->cam_calib_conf_path;
                assert(!this->cam_calib_conf_path.empty());
                cv::FileStorage calib_res = cv::FileStorage(this->cam_calib_conf_path,
                                                            cv::FileStorage::READ);
                calib_res["device"] >> this->cam_path;
                calib_res["camera_matrix"] >> this->cam_mat;
                calib_res["dist_coeffs"] >> this->dist_coeffs;

                calib_res.release();
                /* camera calibration file loaded */

                // check whether calibration file is in sync with system configuration
                std::string cam_path_sys;
                system_conf["Camera"]["calib"]["dataSource"]["uri"] >> cam_path_sys;
                if (std::strcmp(cam_path_sys.c_str(), this->cam_path.c_str()) != 0)
                {
                    this->cam_path = "UNDEFINED";
                }

                system_conf["Camera"]["install"]["height"] >> this->cam_install_height;
                system_conf["Camera"]["install"]["pitch"] >> this->cam_install_pitch;
                this->cam_install_pitch *= DEG2RAD;

                /* Loading perception network layers and weight file */
                system_conf["Camera"]["perceptionConfig"]["YOLOPath"]["cfg"] >> this->yolo_cfg;
                system_conf["Camera"]["perceptionConfig"]["YOLOPath"]["weights"] >> this->yolo_weights;

                /* reading coco names from file */
                std::string coco_name_path;
                system_conf["Camera"]["perceptionConfig"]["YOLOPath"]["names"] >> coco_name_path;
                std::ifstream name_file(coco_name_path);
                std::string name;
                while (std::getline(name_file, name))
                {
                    this->coco_classes.push_back(name);
                }
                name_file.close();
                /* end reading coco names */

                /* assign each class with a color */
                for (size_t coco_class = 1; coco_class <= this->coco_classes.size(); ++coco_class)
                {
                    float h, s, v;
                    h = 2 * M_PI * ((float)coco_class / this->coco_classes.size());
                    s = 0.5 * M_1_PI * h;
                    v = 0.5 * M_1_PI * h;
                    cv::Mat3f hsv(cv::Vec3f(h, s, v));
                    cv::Mat3f rgb;
                    cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);

                    cv::Scalar color(rgb.data[0], rgb.data[1], rgb.data[2]);
                    this->class_colors.push_back(color);
                }

                cv::FileNode filter_cfg = system_conf["Camera"]["perceptionConfig"]["classFilter"];
                if (filter_cfg["enable"].real() == 1)
                {
                    // save the class indices of enabled classes
                    for (cv::FileNodeIterator node = ++filter_cfg.begin(); node != filter_cfg.end(); ++node)
                    {
                        std::string name = (*node).string();
                        for (size_t i = 0; i < this->coco_classes.size(); i++)
                        {
                            std::string class_name = this->coco_classes[i];
                            if (std::strcmp(name.c_str(), class_name.c_str()) == 0)
                            {
                                this->valid_classes.push_back(i);
                                break;
                            }
                        }
                    }
                }
            }

            /* load mmWave Radar parameters */
            system_conf["mmWave"]["radar"]["commandPort"] >> this->radar_cmd_port;
            system_conf["mmWave"]["radar"]["baudRate"] >> this->baud_rate;
            system_conf["mmWave"]["radar"]["profileCfg"] >> this->radar_profile_path;

            /* load radar configuraion commands */
            this->loadRadarProfile();

            /* load DCA1000 configuration */
            system_conf["mmWave"]["DCA1000"]["ip"] >> this->dca_addr;
            system_conf["mmWave"]["DCA1000"]["dataPort"] >> this->dca_data_port;
            system_conf["mmWave"]["DCA1000"]["triggerMode"] >> this->capture_trigger_mode;

            if (this->capture_trigger_mode == 1)
            {
                system_conf["mmWave"]["DCA1000"]["cmdPort"] >> this->dca_cmd_port;
                system_conf["mmWave"]["DCA1000"]["cfgPath"] >> this->dca_conf_path;
            }

            system_conf.release();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return;
    }

    SystemConf::~SystemConf()
    {
    }

    void SystemConf::loadRadarProfile()
    {
        /* find radar model */
        if (this->radar_profile_path.find("6843") != std::string::npos)
        {
            this->radar_model = "IWR6843";
        }
        else if (this->radar_profile_path.find("1642") != std::string::npos)
        {
            this->radar_model = "IWR1642";
        }
        else
        {
            this->radar_model = "UNDEFINED";
        }

        std::ifstream profile_config(this->radar_profile_path);
        std::string command;
        this->radar_cmd_list.clear();
        while (std::getline(profile_config, command))
        {
            // skip comments in profile
            if (command.front() == '%')
            {
                continue;
            }

            std::stringstream cmd;
            cmd << command << "\r\n";
            this->radar_cmd_list.push_back(cmd.str());
        }

        // add sensorStop as final command to stop radar right after configuration
        this->radar_cmd_list.push_back("sensorStop\r\n");
        profile_config.close();

        return;
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

    void getNormMat(Eigen::MatrixXcf &src, Eigen::MatrixXd &dst)
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

        mmfusion::Conv2D(src, dst, conv_kernel);

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
