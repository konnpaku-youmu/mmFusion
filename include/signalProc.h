#ifndef SIGNAL_PROC_H
#define SIGNAL_PROC_H

#include "utilities.h"

#include <gsl/gsl_fft_complex.h>

namespace mmfusion
{
    template<typename MatrixT>
    void FFT(MatrixT src);
}

#endif