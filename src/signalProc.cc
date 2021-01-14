#include "signalProc.h"
namespace mmfusion
{
    SignalProcessor::SignalProcessor(mmfusion::SystemConf &cfg, pthread_mutex_t &mut)
    {
        this->_cfg = &cfg;
        this->_mutex = mut;

        this->virtualAnt = this->_cfg->tx_num * this->_cfg->rx_num;
        this->_output.rw_lock = mmfusion::RWStatus::UNAVAILABLE;

        return;
    }

    SignalProcessor::~SignalProcessor()
    {
        delete this->_cfg;
        delete this->_capture_board;
    }

    void SignalProcessor::entryPoint()
    {
        while (!flag)
        {
            this->_process();
        }
    }

    void SignalProcessor::bindDevice(mmfusion::DCA1000 &dev)
    {
        this->_capture_board = &dev;
        return;
    }

    void SignalProcessor::_process()
    {
        this->_output.rw_lock = mmfusion::RWStatus::WRITING;
        if (this->_capture_board->getRawData(this->_output.raw))
        {
            this->loops = this->_output.raw.cols() / virtualAnt;
            this->adc_samples = this->_output.raw.rows();

            // compute 1D-FFT
            this->_output.fft_1d = Eigen::MatrixXcd::Zero(this->_output.raw.rows(),
                                                          this->_output.raw.cols());
            this->_compute_1d_fft();
            this->_cfar(this->_output.fft_1d);

            // compute 2D-FFT
            this->_output.fft_2d = Eigen::MatrixXcd::Zero(this->_output.fft_1d.rows(),
                                                          this->_output.fft_1d.cols());
            this->_compute_2d_fft();
        }

        this->_output.rw_lock = mmfusion::RWStatus::AVAILABLE;

        return;
    }

    void SignalProcessor::_compute_1d_fft()
    {
#pragma omp parallel
#pragma omp for
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
#pragma omp parallel
#pragma omp for
            for (int chirp = 0; chirp < this->loops; ++chirp)
            {
                double fft_container[2 * this->adc_samples];

                for (int sample = 0; sample < this->adc_samples; ++sample)
                {
                    fft_container[2 * sample] = this->_output.raw(sample, rx * loops + chirp).real();
                    fft_container[2 * sample + 1] = this->_output.raw(sample, rx * loops + chirp).imag();
                }

                /* compute using GSL */
                gsl_fft_complex_radix2_forward(fft_container, 1, this->adc_samples);

                Eigen::VectorXcd fft_result = Eigen::VectorXcd::Zero(this->adc_samples);
                for (int i = 0; i < 2 * this->adc_samples; i += 2)
                {
                    Eigen::dcomplex cmplx(fft_container[i], fft_container[i + 1]);
                    fft_result(i / 2) = cmplx;
                }
                this->_output.fft_1d.col(rx * loops + chirp) = fft_result;
            }
        }

        return;
    }

    void SignalProcessor::_compute_2d_fft()
    {
#pragma omp parallel
#pragma omp for
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
#pragma omp parallel
#pragma omp for
            for (int sample = 0; sample < this->adc_samples; ++sample)
            {
                double fft_container[2 * this->loops];

                // compute DC component
                Eigen::MatrixXcd rx_n_range_k = this->_output.fft_1d.block(sample, rx * this->loops,
                                                                           1, this->loops);
                Eigen::dcomplex dc_sum = rx_n_range_k.sum();

#pragma omp parallel
#pragma omp for
                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    rx_n_range_k(0, chirp) -= dc_sum;
                    fft_container[2 * chirp] = rx_n_range_k(0, chirp).real();
                    fft_container[2 * chirp + 1] = rx_n_range_k(0, chirp).imag();
                }
                /* compute 2D-FFT */
                gsl_fft_complex_radix2_forward(fft_container, 1, this->loops);
#pragma omp parallel
#pragma omp for
                for (int i = 0; i < 2 * this->loops; i += 2)
                {
                    Eigen::dcomplex cmplx(fft_container[i], fft_container[i + 1]);
                    this->_output.fft_2d(sample, rx * loops + (i / 2)) = cmplx;
                }
            }
        }

        return;
    }

    void SignalProcessor::_cfar(Eigen::MatrixXcd &raw_fft)
    {
        Eigen::MatrixXd fft_norm;
        mmfusion::getNormMat(raw_fft, fft_norm);

        this->_output.cfar_1d = Eigen::MatrixXd::Zero(fft_norm.rows(),
                                                      fft_norm.cols());
        int col;
#pragma omp parallel
#pragma omp for private(col)
        for (col = 0; col < fft_norm.cols(); ++col)
        {
            Eigen::VectorXd col_vec = fft_norm.col(col);
            Eigen::VectorXd cfar = mmfusion::cfarConv(col_vec, 11, 1, 2.0);

            this->_output.cfar_1d.col(col) = cfar;
        }

        return;
    }

    bool SignalProcessor::getData(Eigen::MatrixXcd &raw_adc,
                                  Eigen::MatrixXcd &fft1,
                                  Eigen::MatrixXcd &fft2,
                                  Eigen::MatrixXd &cfar)
    {
        bool ret = false;

        if (this->_output.rw_lock == ACCESSED)
        {
        }
        else if (this->_output.rw_lock == AVAILABLE)
        {
            this->_output.rw_lock = mmfusion::RWStatus::READING;
            raw_adc = this->_output.raw;
            fft1 = this->_output.fft_1d;
            fft2 = this->_output.fft_2d;
            cfar = this->_output.cfar_1d;
            this->_output.rw_lock = mmfusion::RWStatus::ACCESSED;
            ret = true;
        }

        return ret;
    }
} // namespace mmfusion