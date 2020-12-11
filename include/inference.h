#ifndef INFER_H
#define INFER_H

#include <iostream>
#include <memory>
#include <unistd.h>
#include <pthread.h>
#include <mutex>
#include <eigen3/Eigen/Eigen>
#include <NvInfer.h>
#include <NvInferRuntime.h>

#include "cam_calib.h"
#include "utilities.h"

namespace mmfusion
{
    class Logger : public nvinfer1::ILogger
    {
        void log(Severity severity, const char *msg) override
        {
            if (severity != Severity::kINFO)
            {
                std::cout << msg << std::endl;
            }
        }
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

    class ImageVis : public MultiThreading
    {
    private:
        cv::Mat *_content;

    protected:
        void entryPoint();
    
    public:
        ImageVis(cv::Mat *, pthread_mutex_t &);

        ~ImageVis();
    };

    class VideoCap : public MultiThreading
    {
    private:
        mmfusion::SystemConf *cfg;

        cv::VideoCapture _cap;

        cv::Mat *_frame;

    protected:
        void entryPoint();

    public:
        VideoCap(mmfusion::SystemConf &, cv::Mat *, pthread_mutex_t &);

        ~VideoCap();
    };

    class DNNInference : public MultiThreading
    {
    private:
        cv::Mat *frame;

        cv::Mat *output;

        mmfusion::SystemConf *_cfg;

        Eigen::Matrix4d _cam_2_world;

        cv::dnn::Net _net;

    protected:
        void entryPoint();

    public:
        DNNInference(mmfusion::SystemConf &, cv::Mat *,
                     cv::Mat *, pthread_mutex_t &);

        ~DNNInference();
    };

    class NvInference : public MultiThreading
    {
    private:
    protected:
    public:
    };

} // namespace mmfusion
#endif