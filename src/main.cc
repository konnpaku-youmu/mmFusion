#include "utilities.h"
#include "cam_calib.h"
#include "inference.h"
#include "mmWave.h"
#include "signalProc.h"
#include "visualize.h"

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

    mmfusion::DCA1000 data_cap(cfg, mut);

    radar.startThread(attr);
    data_cap.startThread(attr);

    mmfusion::SignalProcessor sp;
    sp.bindDevice(data_cap);
    sp.startThread(attr);

    /* Start GUI */
    mmfusion::DataPlotWrapper plt(argc, argv, radar, sp);
    plt.show();

    pthread_attr_destroy(&attr);
    pthread_mutex_destroy(&mut);
    pthread_exit(NULL);

    return 0;
}
