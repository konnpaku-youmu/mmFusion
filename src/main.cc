#include "cam_calib.h"
// #include "inference.h"
#include "mmWave.h"

int main(int argc, char **argv)
{
    mmfusion::SystemConf cfg("/home/hcrd/Projects/mmFusion/config/system.xml");

    mmfusion::Radar radar(cfg);
    // if (cfg.camera_mat.empty() || cfg.dist_coeffs.empty() || std::strcmp(cfg.device.c_str(), "none") == 0)
    // {
    //     std::cout << "Calibration not found or corrupted..." << std::endl;
    //     mmfusion::Calibration calib("/home/hcrd/Projects/mmFusion/config/system.xml");
    //     calib.runAndSave();
    // }
    // else
    // {
    //     cv::Mat frame, dst;

    //     pthread_mutex_t mutex_frame;
    //     pthread_attr_t attr;
    //     pthread_mutex_init(&mutex_frame, NULL);
    //     pthread_attr_init(&attr);
    //     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    //     mmfusion::VideoCap cap(cfg, &frame, mutex_frame);
    //     mmfusion::DNNInference infer_dnn(cfg, &frame, &dst, mutex_frame);

    //     cap.startThread(attr);
    //     infer_dnn.startThread(attr);

    //     while(true);

    //     infer_dnn.stopThread();
    //     cap.stopThread();

    //     pthread_attr_destroy(&attr);
    //     pthread_mutex_destroy(&mutex_frame);
    //     pthread_exit(NULL);
    // }

    return 0;
}
