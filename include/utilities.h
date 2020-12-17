#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <fstream>
#include <signal.h>
#include <opencv4/opencv2/opencv.hpp>

namespace mmfusion
{
    struct SystemConf
    {
        // camera related
        std::string device;
        double camera_height;
        double camera_pitch;
        std::string calib_conf_path;
        cv::Mat camera_mat;
        cv::Mat dist_coeffs;
        
        // inference related
        std::string yolo_cfg;
        std::string yolo_weights;
        std::vector<std::string> coco_classes;
        std::vector<cv::Scalar> class_colors;
        std::vector<int> valid_classes;

        // mmWave related
        std::string cmd_port;
        int baud_rate;
        std::vector<std::string> cmd_list;
        
        // DCA1000 related
        
        SystemConf(const std::string &);
    };

    struct DetectedObj
    {
        int classID = -1;
        float confidence = -1;
        cv::Rect bbox;
    };
    
    int argmax(cv::Mat1d);

    void padding(cv::Mat &, cv::Mat &);
} // namespace mmfusion

#endif