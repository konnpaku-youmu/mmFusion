#include "signalProc.h"
namespace mmfusion
{
    SignalProcessor::SignalProcessor()
    {
        this->_frame_1d_fft.rw_lock = mmfusion::RWStatus::UNAVAILABLE;
    }

    SignalProcessor::~SignalProcessor()
    {
    }

    void SignalProcessor::entryPoint()
    {
        for (;;)
        {
            this->_compute_1d_fft();
        }
    }

    void SignalProcessor::bindDevice(mmfusion::DCA1000 &dev)
    {
        this->_capture_board = &dev;

        return;
    }

    void SignalProcessor::_compute_1d_fft()
    {
        this->_frame_1d_fft.rw_lock = mmfusion::RWStatus::WRITING;
        if (this->_capture_board->getRawData(this->_raw_data))
        {
            this->_raw_bypass.rw_lock = mmfusion::RWStatus::WRITING;
            this->_raw_bypass.data_flattened = this->_raw_data;

            this->_frame_1d_fft.coeffs = Eigen::MatrixXcd::Zero(this->_raw_data.rows(), 32);
            for (size_t chirp = 0; chirp < 32; ++chirp)
            {
                double single_chirp[2 * this->_raw_data.rows()];
                Eigen::VectorXcd fft_res = Eigen::VectorXcd::Zero(this->_raw_data.rows());

                for (size_t sample = 0; sample < this->_raw_data.rows(); ++sample)
                {
                    single_chirp[2 * sample] = this->_raw_data(sample, chirp).real();
                    single_chirp[2 * sample + 1] = this->_raw_data(sample, chirp).imag();
                }

                /* compute FFT */
                gsl_fft_complex_radix2_forward(single_chirp, 1, this->_raw_data.rows());
                for (size_t i = 0; i < 2 * this->_raw_data.rows(); i += 2)
                {
                    Eigen::dcomplex cplx(single_chirp[i], single_chirp[i + 1]);
                    fft_res(i / 2) = cplx;
                }
                this->_frame_1d_fft.coeffs.col(chirp) = fft_res;
            }

            this->_raw_bypass.rw_lock = mmfusion::RWStatus::AVAILABLE;
        }
        this->_frame_1d_fft.rw_lock = mmfusion::RWStatus::AVAILABLE;
        return;
    }

    bool SignalProcessor::getData(Eigen::MatrixXcd &raw_data, Eigen::MatrixXcd &fft)
    {
        bool ret = false;

        if (this->_frame_1d_fft.rw_lock == ACCESSED)
        {
            ret = false;
        }
        else if (this->_frame_1d_fft.rw_lock == AVAILABLE &&
                 this->_raw_bypass.rw_lock == AVAILABLE &&
                 this->_frame_1d_fft.coeffs.rows() > 0 &&
                 this->_frame_1d_fft.coeffs.cols() > 0)
        {
            this->_frame_1d_fft.rw_lock = mmfusion::RWStatus::READING;
            fft = this->_frame_1d_fft.coeffs;
            raw_data = this->_raw_bypass.data_flattened;
            this->_frame_1d_fft.rw_lock = mmfusion::RWStatus::ACCESSED;
            ret = true;
        }

        return ret;
    }
} // namespace mmfusion