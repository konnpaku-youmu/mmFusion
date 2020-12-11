#ifndef CAM_CALIB
#define CAM_CALIB

#include <iostream>
#include <chrono>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv4/opencv2/cudawarping.hpp>
#include <opencv4/opencv2/dnn/dnn.hpp>

namespace mmfusion
{
    struct
    {
        int frame_num;
        std::string pattern;
        cv::Size pattern_size;
        cv::Size image_size;
        float spacing = 0;
        std::string source;
        std::string device;
        std::string save_path;
    } typedef CalibParas;

    enum STATUS
    {
        CAPTURE,
        CALIB,
        DONE
    };

    class Calibration
    {
    private:
        cv::FileStorage _config;

        CalibParas _params;

        STATUS status;

        std::vector<std::vector<cv::Point2f>> _corner_pixels;

        std::vector<std::vector<cv::Point3f>> _obj_points;

        cv::Mat _camera_mat;

        cv::Mat _dist_coeffs;

        double _reproj_err;

        void _capture(cv::Mat &);

        void _calib();

        void _show_undistorted(cv::Mat &);

        void _calc_corner_position(cv::Size, float,
                                   std::vector<cv::Point3f> &);

    public:
        Calibration(const std::string &);

        ~Calibration();

        void runAndSave();
    };

    void undistortGpu(cv::Mat &, cv::Mat &,
                      cv::InputArray &, cv::InputArray &);

} // namespace mmfusion

#endif
