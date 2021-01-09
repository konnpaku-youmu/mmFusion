#ifndef SIGNAL_PROC_H
#define SIGNAL_PROC_H

#include "utilities.h"
#include "mmWave.h"

#include <gsl/gsl_fft_complex.h>

namespace mmfusion
{
    struct
    {
        Eigen::MatrixXcd coeffs;
        mmfusion::RWStatus rw_lock;
    } typedef FFTData;

    class SignalProcessor : public MultiThreading
    {
    private:
        mmfusion::DCA1000 *_capture_board;

        Eigen::MatrixXcd _raw_data;

        OrganizedADCData _raw_bypass;

        FFTData _frame_1d_fft;

        void _compute_1d_fft();

    protected:
        void entryPoint();

    public:
        SignalProcessor();

        ~SignalProcessor();

        void bindDevice(mmfusion::DCA1000 &);

        bool getData(Eigen::MatrixXcd &, Eigen::MatrixXcd &);
    };
} // namespace mmfusion

#endif