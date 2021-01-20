#ifndef SIGNAL_PROC_H
#define SIGNAL_PROC_H

#include "utilities.h"
#include "mmWave.h"

#ifdef WITH_CUDA
#include <cufft.h>
#else
#include <gsl/gsl_fft_complex.h>
#endif

namespace mmfusion
{
    struct
    {
        Eigen::MatrixXcd raw;
        Eigen::MatrixXcd fft_1d;
        Eigen::MatrixXd cfar_1d;
        Eigen::MatrixXcd fft_2d;
        mmfusion::RWStatus rw_lock;
    } typedef ProcOutput;

    class SignalProcessor : public MultiThreading
    {
    private:
        mmfusion::SystemConf *_cfg;

        mmfusion::DCA1000 *_capture_board;

        int virtualAnt = 0, loops = 0, adc_samples = 0;

#ifdef WITH_CUDA
        cufftHandle *_plan;
#endif

        ProcOutput _output;

        void _process();

        void _compute_1d_fft();

        void _compute_2d_fft();

        void _cfar(Eigen::MatrixXcd &);

    protected:
        void entryPoint();

    public:
        SignalProcessor(mmfusion::SystemConf &,
                        pthread_mutex_t &);

        ~SignalProcessor();

        void bindDevice(mmfusion::DCA1000 &);

        bool getData(Eigen::MatrixXcd &, Eigen::MatrixXcd &,
                     Eigen::MatrixXcd &, Eigen::MatrixXd &);
    };
} // namespace mmfusion

#endif