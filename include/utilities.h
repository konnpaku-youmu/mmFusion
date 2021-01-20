#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <fstream>
#include <memory>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <mutex>
#include <omp.h>
#include <complex>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include <eigen3/Eigen/Eigen>
#include <opencv4/opencv2/opencv.hpp>

#ifdef WITH_CUDA
#include <cuda_runtime.h>
#include <cuda.h>
#endif

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

    enum RWStatus
    {
        UNAVAILABLE,
        WRITING,
        READING,
        AVAILABLE,
        ACCESSED
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
        int tx_num = 0, rx_num = 0;
        int adc_samples = 0;
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
        uint8_t flag = 0;

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

    void getNormMat(Eigen::MatrixXcd &, Eigen::MatrixXd &);

    Eigen::VectorXd cfarConv(Eigen::VectorXd &, int window_size = 9,
                             int stride = 1, double threshold = 3.0);

    void blur2D(Eigen::MatrixXd &, Eigen::MatrixXd &, int radius = 7);

    void Conv2D(Eigen::MatrixXd &, Eigen::MatrixXd &, Eigen::MatrixXd &);

} // namespace mmfusion

#endif