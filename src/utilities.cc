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

            cfg["mmWave"]["radar"]["commandPort"] >> this->cmd_port;
            cfg["mmWave"]["radar"]["baudRate"] >> this->baud_rate;

            std::string mmwave_profile;
            cfg["mmWave"]["radar"]["profileCfg"] >> mmwave_profile;
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
            profile_config.close();

            cfg.release();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    int argmax(cv::Mat1d list)
    {
        float max = -1;
        int idx = -1;
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
} // namespace mmfusion
