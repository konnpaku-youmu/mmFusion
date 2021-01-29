#ifndef UTILITIES_H
#define UTILITIES_H

// #define WITH_CUDA

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

#define DEG2RAD 3.14159265 / 180

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

    class SystemConf
    {
    public:
        SystemConf(const std::string &);
        
        ~SystemConf();

        /**
         * @brief To load initializing commands from radar profile as
         *        a list of strings. This method can be called while 
         *        the radar is alarmed when paused.
         *
         */
        void loadRadarProfile();

        /* camera related params */
        std::string cam_path;
        double cam_install_height;
        double cam_install_pitch;
        std::string cam_calib_conf_path;
        cv::Mat cam_mat;
        cv::Mat dist_coeffs;

        /* visual inference params */
        std::string yolo_cfg;
        std::string yolo_weights;
        std::vector<std::string> coco_classes; // coco class name list
        std::vector<cv::Scalar> class_colors; // colors for visualizing color classes
        std::vector<int> valid_classes; // coco class id to display

        /* mmWave radar configuration params */
        std::string radar_model;
        std::string radar_cmd_port; // serial port path
        int baud_rate;
        std::string radar_profile_path;
        std::vector<std::string> radar_cmd_list; // commands to be sent to radar

        /* mmWave radar runtime params */
        uint8_t tx_num;
        uint8_t rx_num;
        uint16_t adc_samples;
        uint16_t chirp_loops;

        /* data capture board params */
        std::string dca_addr;
        uint16_t dca_data_port;
        uint8_t capture_trigger_mode;
        uint16_t dca_cmd_port; // for software configuration and trigger mode
        std::string dca_conf_path; // for software configuration and trigger mode
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

    void getNormMat(Eigen::MatrixXcf &, Eigen::MatrixXd &);

    Eigen::VectorXd cfarConv(Eigen::VectorXd &, int window_size = 9,
                             int stride = 1, double threshold = 3.0);

    void blur2D(Eigen::MatrixXd &, Eigen::MatrixXd &, int radius = 5);

    void Conv2D(Eigen::MatrixXd &, Eigen::MatrixXd &, Eigen::MatrixXd &);

} // namespace mmfusion

#endif