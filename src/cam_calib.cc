#include "cam_calib.h"

namespace mmfusion
{
    std::ostream &operator<<(std::ostream &os, const CalibParas &paras)
    {
        os << "Calibration Configuration" << std::endl;
        os << "---" << std::endl;
        os << "Number of frames: " << paras.frame_num << std::endl;
        os << "Pattern: " << paras.pattern << std::endl;
        os << "Size: " << paras.pattern_size << paras.spacing << std::endl;
        os << "Device Type: " << paras.source << std::endl;
        os << "Name:" << paras.device << std::endl;
        os << "---" << std::endl;

        return os;
    }

    Calibration::Calibration(const std::string &cfg_path)
    {
        this->_config = cv::FileStorage(cfg_path, cv::FileStorage::READ);
        this->_config["Camera"]["calib"]["frameNum"] >> this->_params.frame_num;
        this->_config["Camera"]["calib"]["pattern"] >> this->_params.pattern;
        this->_config["Camera"]["calib"]["width"] >> this->_params.pattern_size.width;
        this->_config["Camera"]["calib"]["height"] >> this->_params.pattern_size.height;
        this->_config["Camera"]["calib"]["spacing"] >> this->_params.spacing;
        this->_config["Camera"]["calib"]["dataSource"]["type"] >> this->_params.source;
        this->_config["Camera"]["calib"]["dataSource"]["uri"] >> this->_params.device;
        this->_config["Camera"]["calib"]["savePath"] >> this->_params.save_path;

        assert(this->_params.frame_num >= 5);
        assert((std::strcmp(this->_params.pattern.c_str(), "Chessboard") == 0) ||
               (std::strcmp(this->_params.pattern.c_str(), "CircleGrid") == 0));
        assert(this->_params.pattern_size.width > 0 && this->_params.pattern_size.height > 0 &&
               (this->_params.pattern_size.width + this->_params.pattern_size.height) % 2 == 1);
        assert(this->_params.spacing > 0);
        assert((std::strcmp(this->_params.source.c_str(), "webcam") == 0) ||
               (std::strcmp(this->_params.source.c_str(), "stream") == 0));
        // display parameters
        std::cout << this->_params << std::endl;

        this->status = CAPTURE;
    }

    Calibration::~Calibration()
    {
    }

    void Calibration::runAndSave()
    {
        cv::VideoCapture cap(this->_params.device);
        uint8_t key_pressed = 0;
        cv::Mat frame;

        while (key_pressed != (int)'q')
        {
            cap >> frame;
            if (this->_params.image_size == cv::Size(0, 0))
            {
                this->_params.image_size = frame.size();
            }

            cv::imshow("Raw Image", frame);
            switch (this->status)
            {
            case CAPTURE:
                if (key_pressed == (int)'c')
                {
                    this->_capture(frame);
                }
                break;
            case CALIB:
                this->_calib();
                break;
            case DONE:
                this->_show_undistorted(frame);
                break;
            default:
                break;
            }
            key_pressed = cv::waitKey(20);
        }

        cv::destroyAllWindows();
        cap.release();
    }

    void Calibration::_capture(cv::Mat &frame)
    {
        cv::Mat frame_mono;
        cv::cvtColor(frame, frame_mono, cv::COLOR_BGR2GRAY);
        bool pattern_found;
        std::vector<cv::Point2f> markers;

        pattern_found = cv::findChessboardCorners(frame, this->_params.pattern_size, markers,
                                                  cv::CALIB_CB_ADAPTIVE_THRESH +
                                                      cv::CALIB_CB_NORMALIZE_IMAGE +
                                                      cv::CALIB_CB_FAST_CHECK);
        if (pattern_found)
        {
            cv::cornerSubPix(frame_mono, markers, cv::Size(5, 5), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            this->_corner_pixels.push_back(markers);
            std::vector<cv::Point3f> new_obj_pts;
            this->_calc_corner_position(this->_params.pattern_size,
                                        this->_params.spacing, new_obj_pts);
            this->_obj_points.push_back(new_obj_pts);

            // show image with corners
            std::stringstream info;
            info << "Capturing..." << this->_corner_pixels.size() << "/" << this->_params.frame_num;
            cv::putText(frame, info.str(), cv::Point2d(0.75 * frame.size().width, 0.9 * frame.size().height),
                        cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 170, 250), 1, cv::LINE_AA);
            cv::drawChessboardCorners(frame, this->_params.pattern_size,
                                      cv::Mat(markers), pattern_found);
            cv::imshow("Chessboard", frame);

            if (this->_corner_pixels.size() == this->_params.frame_num)
            {
                this->status = CALIB;
            }
        }
        else
        {
            std::cout << "Cannot find calibration board" << std::endl;
        }
        return;
    }

    void Calibration::_calib()
    {
        std::vector<cv::Mat> R, t;
        this->_reproj_err = cv::calibrateCamera(this->_obj_points, this->_corner_pixels,
                                                this->_params.image_size, this->_camera_mat,
                                                this->_dist_coeffs, R, t);

        cv::FileStorage calib_res(this->_params.save_path, cv::FileStorage::WRITE);
        
        if (this->_reproj_err <= 0.5)
        {
            auto now = std::chrono::system_clock::now();
            std::time_t now_t = std::chrono::system_clock::to_time_t(now);

            calib_res << "calibration_timestamp" << std::ctime(&now_t);
            calib_res << "device" << this->_params.device;
            calib_res << "camera_matrix" << this->_camera_mat;
            calib_res << "dist_coeffs" << this->_dist_coeffs;
            calib_res << "reprojection_err" << _reproj_err;

            this->status = DONE;
            cv::destroyAllWindows();
        }
        else
        {
            std::cout << "Re-projection_error = " << _reproj_err << " is out of bound...Retry";
            this->_obj_points.clear();
            this->_corner_pixels.clear();
            this->status = CAPTURE;
        }
        calib_res.release();
        return;
    }

    void Calibration::_show_undistorted(cv::Mat &frame)
    {
        cv::Mat K, dist, undistorted;
        double reproj;

        cv::FileStorage calib_res(this->_params.save_path, cv::FileStorage::READ);
        calib_res["camera_matrix"] >> K;
        calib_res["dist_coeffs"] >> dist;
        calib_res["reprojection_err"] >> reproj;
        calib_res.release();

        undistortGpu(frame, undistorted, K, dist);

        std::stringstream info;
        info << "Calibrated! Re-projection Error = " << reproj;
        cv::putText(frame, info.str(), cv::Point(0.7 * frame.size().width, 0.9 * frame.size().height),
                    cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(10, 200, 30), 1, cv::LINE_AA);
        cv::imshow("Undistorted", undistorted);

        return;
    }

    void Calibration::_calc_corner_position(cv::Size pattern_size, float spacing,
                                            std::vector<cv::Point3f> &obj_points)
    {
        obj_points.clear();
        for (size_t i = 0; i < pattern_size.height; ++i)
        {
            for (size_t ii = 0; ii < pattern_size.width; ++ii)
            {
                obj_points.push_back(cv::Point3f(ii * spacing, i * spacing, 0));
            }
        }
        return;
    }

    void undistortGpu(cv::Mat &src, cv::Mat &dst,
                      cv::InputArray &camMat, cv::InputArray &distCoeffs)
    {
        cv::Mat map1, map2;

        cv::initUndistortRectifyMap(camMat, distCoeffs, cv::Mat(), camMat,
                                    src.size(), CV_32FC1, map1, map2);

        cv::cuda::GpuMat map1_gpu(map1);
        cv::cuda::GpuMat map2_gpu(map2);
        cv::cuda::GpuMat src_gpu(src);
        cv::cuda::GpuMat dst_gpu(src.size(), src.type());

        cv::cuda::remap(src_gpu, dst_gpu, map1_gpu, map2_gpu, cv::INTER_CUBIC);
        dst_gpu.download(dst);

        return;
    }

} // namespace mmfusion