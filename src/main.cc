#include "utilities.h"
#include "cam_calib.h"
#include "inference.h"
#include "mmWave.h"
#include "signalProc.h"

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

    Eigen::MatrixXcd adc_mat;
    mmfusion::DCA1000 data_cap(cfg, mut);

    radar.startThread(attr);
    data_cap.startThread(attr);

    for (;;)
    {
        // read raw adc data
        if (data_cap.getRawData(adc_mat))
        {
            Eigen::MatrixXcd rx_0 = Eigen::MatrixXcd::Map(adc_mat.data(),
                                                          adc_mat.rows(), adc_mat.cols() / 4,
                                                          Eigen::OuterStride<>(4 * adc_mat.rows()));
            // std::ofstream file("../rx_0_chirp.tsv");
            // file << rx_0.transpose().format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, "\t ", "\n"));
            // file.close();
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
