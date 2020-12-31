#include "utilities.h"
#include "cam_calib.h"
#include "inference.h"
#include "mmWave.h"

int main(int argc, char **argv)
{
    mmfusion::SystemConf cfg("/home/hcrd/Projects/mmFusion/config/system.xml");

    /* Setting up mutex for multi-threading*/
    pthread_mutex_t mut;
    pthread_attr_t attr;
    pthread_mutex_init(&mut, NULL);
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /* Instantiate Devices */

    mmfusion::Radar radar(cfg, mut);
    radar.configure();

    std::vector<Eigen::MatrixXcd> adc_mat;
    mmfusion::DCA1000 data_cap(cfg, mut);

    radar.startThread(attr);
    data_cap.startThread(attr);

    for (;;)
    {
        if (data_cap.getStatus() == mmfusion::deviceStatus::RUNNING)
        {
            data_cap.organize(adc_mat);
        }
    }

    pthread_attr_destroy(&attr);
    pthread_mutex_destroy(&mut);
    pthread_exit(NULL);

    // if (cfg.camera_mat.empty() || cfg.dist_coeffs.empty() || std::strcmp(cfg.device.c_str(), "none") == 0)
    // {
    //     std::cout << "Calibration not found or corrupted..." << std::endl;
    //     mmfusion::Calibration calib("/home/hcrd/Projects/mmFusion/config/system.xml");
    //     calib.runAndSave();
    // }
    // else
    // {
    // }

    return 0;
}
