#include "inference.h"

namespace mmfusion
{
    bool MultiThreading::startThread(pthread_attr_t &attr)
    {
        return (pthread_create(&(this->_thread), &attr, _internal_thread_entry, this) == 0);
    }

    void MultiThreading::stopThread()
    {
        return (void)pthread_join(this->_thread, NULL);
    }

    void *MultiThreading::_internal_thread_entry(void *This)
    {
        pthread_mutex_lock(&(((MultiThreading *)This)->_mutex));
        ((MultiThreading *)This)->entryPoint();
        pthread_mutex_unlock(&(((MultiThreading *)This)->_mutex));

        return NULL;
    }

    ImageVis::ImageVis(cv::Mat *_img_ptr, pthread_mutex_t &mutex)
    {
        this->_content = _img_ptr;
        this->_mutex = mutex;
        return;
    }

    ImageVis::~ImageVis()
    {
        cv::destroyAllWindows();
    }

    void ImageVis::entryPoint()
    {
        while (true)
        {
            if (this->_content->size() == cv::Size(0, 0))
            {
                continue;
            }
            else
            {
                cv::imshow("DNNInfer", *(this->_content));
                cv::waitKey(100);
            }
        }
    }

    VideoCap::VideoCap(mmfusion::SystemConf &cfg, cv::Mat *frame, pthread_mutex_t &mutex)
    {
        this->cfg = &cfg;
        this->_frame = frame;
        this->_mutex = mutex;
        this->_cap.open(this->cfg->device);

        return;
    }

    VideoCap::~VideoCap()
    {
    }

    void VideoCap::entryPoint()
    {
        while (true)
        {
            this->_cap >> *(this->_frame);
        }

        return;
    }

    DNNInference::DNNInference(mmfusion::SystemConf &cfg, cv::Mat *_src,
                               cv::Mat *_out, pthread_mutex_t &mutex)
    {
        this->_cfg = &cfg;
        this->frame = _src;
        this->output = _out;
        this->_mutex = mutex;

        this->_cam_2_world << cos(cfg.camera_pitch), 0, sin(cfg.camera_pitch), 0,
            0, 1, 0, 0,
            -sin(cfg.camera_pitch), 0, cos(cfg.camera_pitch), cfg.camera_height,
            0, 0, 0, 1;

        this->_net = cv::dnn::readNetFromDarknet(cfg.yolo_cfg, cfg.yolo_weights);
        this->_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        this->_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

        return;
    }

    DNNInference::~DNNInference()
    {
        // this->_out->release();
    }

    void DNNInference::entryPoint()
    {
        while (cv::waitKey(20))
        {
            if (this->frame->size() == cv::Size(0, 0))
            {
                continue;
            }

            cv::Rect roi;
            cv::Mat optimal_k = cv::getOptimalNewCameraMatrix(this->_cfg->camera_mat, this->_cfg->dist_coeffs,
                                                              this->frame->size(), 0.1, this->frame->size(), &roi, true);
            double f_x, f_y, c_x, c_y;
            f_x = optimal_k.at<double>(0, 0);
            f_y = optimal_k.at<double>(1, 1);
            c_x = optimal_k.at<double>(0, 2);
            c_y = optimal_k.at<double>(1, 2);

            mmfusion::undistortGpu(*(this->frame), *(this->output), optimal_k, this->_cfg->dist_coeffs);
            *(this->output) = (*(this->output))(roi);

            // padding
            cv::Mat undist_square;
            mmfusion::padding(*(this->output), undist_square);

            // inference
            auto blob = cv::dnn::blobFromImage(undist_square, 0.00392, cv::Size(416, 416),
                                               cv::Scalar(0, 0, 0), true, false);
            this->_net.setInput(blob);

            cv::Mat detection_list;
            this->_net.forward(detection_list);

            std::vector<cv::Rect> boxes;
            std::vector<float> confs;
            std::vector<mmfusion::DetectedObj> objects;

            for (size_t idx = 0; idx < detection_list.rows; ++idx)
            {
                cv::Mat detection = detection_list.row(idx);

                mmfusion::DetectedObj new_obj;

                float center_x, center_y, width, height;
                cv::Range classify_res(5, (detection.cols - 1));
                cv::Mat1d confidence_list = detection.colRange(classify_res);

                new_obj.classID = mmfusion::argmax(confidence_list);
                new_obj.confidence = confidence_list.at<double>(new_obj.classID);

                center_x = (int)(detection.at<float>(0) * undist_square.cols);
                center_y = (int)(detection.at<float>(1) * undist_square.rows);

                cv::Rect2d bbx;

                bbx.width = (int)(detection.at<float>(2) * undist_square.cols);
                bbx.height = (int)(detection.at<float>(3) * undist_square.rows);
                bbx.x = (int)(center_x - (bbx.width / 2));
                bbx.y = (int)(center_y - (bbx.height / 2));
                new_obj.bbox = bbx;

                objects.push_back(new_obj);
                boxes.push_back(bbx);
                confs.push_back(new_obj.confidence);
            }

            std::vector<int> indices;
            cv::dnn::NMSBoxes(boxes, confs, 0.3, 0.55, indices);

            for (auto idx : indices)
            {
                std::stringstream info;
                DetectedObj *obj = &objects[idx];

                // unpadding
                if (undist_square.size().width == this->output->size().width)
                {
                    obj->bbox.y -= (int)((undist_square.size().height - this->output->size().height) / 2);
                }
                else if (undist_square.size().height == this->output->size().height)
                {
                    obj->bbox.x -= (int)((undist_square.size().width - this->output->size().width) / 2);
                }

                int obj_cx, obj_cy;
                obj_cx = obj->bbox.x + (obj->bbox.width / 2);
                obj_cy = obj->bbox.y + (obj->bbox.height / 2);

                double yaw_pix_cam, pitch_pix_cam;
                yaw_pix_cam = atan((obj_cx - c_x) / f_x);
                pitch_pix_cam = atan((c_y - obj_cy) / f_y);

                Eigen::Vector3d P, p;
                p << obj_cx, obj_cy, 1;

                Eigen::Matrix4d _p, _y, T_p_w;
                Eigen::Vector3d dir = Eigen::Vector3d::UnitX();

                _p << cos(pitch_pix_cam), 0, sin(pitch_pix_cam), 0,
                    0, 1, 0, 0,
                    -sin(pitch_pix_cam), 0, cos(pitch_pix_cam), 0,
                    0, 0, 0, 1;
                _y << cos(yaw_pix_cam), sin(yaw_pix_cam), 0, 0,
                    -sin(yaw_pix_cam), cos(yaw_pix_cam), 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
                T_p_w = this->_cam_2_world * _y * _p;

                dir = T_p_w.block<3, 3>(0, 0) * dir;
                double lbd, x, y;
                lbd = (this->_cam_2_world(2, 3) - 1) / dir(2);
                dir = lbd * dir;

                cv::rectangle(*(this->output), obj->bbox, this->_cfg->class_colors[obj->classID], 2, cv::LINE_AA);
                info << this->_cfg->coco_classes[obj->classID] << ':' << std::setprecision(2) << obj->confidence;
                cv::putText(*(this->output), info.str(), obj->bbox.tl(), cv::FONT_HERSHEY_DUPLEX,
                            0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

                std::stringstream ss;
                ss << std::setprecision(2) << "(" << dir(0) << ", " << dir(1) << ")";
                cv::putText(*(this->output), ss.str(), cv::Point(obj_cx - (obj->bbox.width / 2.5), obj_cy), cv::FONT_HERSHEY_DUPLEX,
                            0.3, cv::Scalar(0, 150, 255), 1, cv::LINE_AA);
            }
            cv::imshow("Inference", *(this->output));
        }

        return;
    }

} // namespace mmfusion
