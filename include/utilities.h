#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <fstream>
#include <signal.h>
#include <eigen3/Eigen/Eigen>
#include <armadillo>
#include <complex>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <opencv4/opencv2/opencv.hpp>

using namespace boost::asio::ip;

namespace mmfusion
{
    enum deviceStatus
    {
        INIT,
        CONFIGURED,
        RUNNING,
        FAILED
    };

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
        std::string radar_model;
        std::string cmd_port;
        int baud_rate;
        std::vector<std::string> cmd_list;
        
        // DCA1000 related
        std::string dca_addr;
        int dca_data_port;
        int dca_cmd_port;
        std::string trigger_mode;
        std::string dca_cfg_path;
        
        SystemConf(const std::string &);
    };

    class MultiThreading
    {
    private:
        pthread_t _thread;

        static void *_internal_thread_entry(void *);

    protected:
        pthread_mutex_t _mutex;

        virtual void entryPoint() = 0;

    public:
        MultiThreading() {}

        bool startThread(pthread_attr_t &);

        void stopThread();
    };

    class Device
    {
    protected:
        mmfusion::SystemConf *_cfg;

        mmfusion::deviceStatus _status;
    
    public:
        Device() {}

        virtual void configure() = 0;
    };
    
    struct DetectedObj
    {
        int classID = -1;
        float confidence = -1;
        cv::Rect bbox;
    };

    void waitBeforeContinue();

    int argmax(cv::Mat1d);

    void padding(cv::Mat &, cv::Mat &);

    std::vector<std::string> split(std::string &, const std::string &);
} // namespace mmfusion

#endif