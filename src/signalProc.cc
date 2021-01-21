#include "signalProc.h"
namespace mmfusion
{
    SignalProcessor::SignalProcessor(mmfusion::SystemConf &cfg, pthread_mutex_t &mut)
    {
        this->_cfg = &cfg;
        this->_mutex = mut;

        this->virtualAnt = this->_cfg->tx_num * this->_cfg->rx_num;
        this->adc_samples = this->_cfg->adc_samples;
        this->loops = this->_cfg->loops;
        this->_output.rw_lock = mmfusion::RWStatus::UNAVAILABLE;

#ifdef WITH_CUDA
        // CUDA device query
        int deviceCnt = 0;
        cudaError_t error_id = cudaGetDeviceCount(&deviceCnt);
        if (error_id != cudaSuccess)
        {
        }
        else
        {
            std::cout << "Found " << deviceCnt << " CUDA device(s)" << std::endl;
        }

        if (deviceCnt == 0)
        {
            std::cout << "There's no CUDA device" << std::endl;
        }

        // Initializing cuFFT plan
        this->_plan_1d = new cufftHandle;
        this->_plan_2d = new cufftHandle;
        if (cufftPlan1d(this->_plan_1d, this->adc_samples, CUFFT_C2C, this->loops * this->virtualAnt) != CUFFT_SUCCESS ||
            cufftPlan1d(this->_plan_2d, this->loops, CUFFT_C2C, this->adc_samples * this->virtualAnt) != CUFFT_SUCCESS)
        {
            std::cout << "CUFFT error: Plan creation failed..." << std::endl;
        }

#endif
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
#ifdef WITH_CUDA
        cufftDestroy(*this->_plan_1d);
        cufftDestroy(*this->_plan_2d);
#endif
        return;
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
            assert(this->loops == this->_output.raw.cols() / virtualAnt);
            assert(this->adc_samples == this->_output.raw.rows());

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
#ifdef WITH_CUDA
        cufftComplex *data;
        size_t fftTotalSize = sizeof(cufftComplex) * this->loops * this->virtualAnt * this->adc_samples;
        float2 *sig = new float2[this->loops * this->virtualAnt * this->adc_samples];

        cudaMalloc((void **)&data, fftTotalSize);
        if (cudaGetLastError() != cudaSuccess)
        {
            std::cerr << "CUDA Error: malloc..." << std::endl;
        }
#else
#pragma omp parallel
#pragma omp for
#endif
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
#ifndef WITH_CUDA
#pragma omp parallel
#pragma omp for
#endif
            for (int chirp = 0; chirp < this->loops; ++chirp)
            {
#ifdef WITH_CUDA
                for (int sample = 0; sample < this->adc_samples; ++sample)
                {
                    /* flatten ADC matrix */
                    int idx = (rx * this->loops * this->adc_samples) +
                              (chirp * this->adc_samples) + sample;
                    sig[idx].x = (float)this->_output.raw(sample, rx * loops + chirp).real();
                    sig[idx].y = (float)this->_output.raw(sample, rx * loops + chirp).imag();
                }
#else
                Eigen::VectorXcd fft_result = Eigen::VectorXcd::Zero(this->adc_samples);
                double fft_container[2 * this->adc_samples];

                for (int sample = 0; sample < this->adc_samples; ++sample)
                {
                    fft_container[2 * sample] = this->_output.raw(sample, rx * loops + chirp).real();
                    fft_container[2 * sample + 1] = this->_output.raw(sample, rx * loops + chirp).imag();
                }

                /* compute using GSL */
                gsl_fft_complex_radix2_forward(fft_container, 1, this->adc_samples);

                for (int i = 0; i < 2 * this->adc_samples; i += 2)
                {
                    Eigen::dcomplex cmplx(fft_container[i], fft_container[i + 1]);
                    fft_result(i / 2) = cmplx;
                }
                this->_output.fft_1d.col(rx * loops + chirp) = fft_result;
#endif
            }
        }
#ifdef WITH_CUDA
        cudaMemcpy(data, sig, fftTotalSize, cudaMemcpyHostToDevice);

        // run FFT with CUDA
        if (cufftExecC2C(*this->_plan_1d, data, data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }
        if (cudaDeviceSynchronize() != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        cudaMemcpy(sig, data, fftTotalSize, cudaMemcpyDeviceToHost);

        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
            for (int chirp = 0; chirp < this->loops; ++chirp)
            {
                for (int sample = 0; sample < this->adc_samples; ++sample)
                {
                    /* reconstruct 1D-FFT matrix */
                    int idx = (rx * this->adc_samples * this->loops) +
                              (chirp * this->adc_samples) + sample;
                    Eigen::dcomplex cmplx((double)sig[idx].x, (double)sig[idx].y);
                    this->_output.fft_1d(sample, rx * loops + chirp) = cmplx;
                }
            }
        }

        cudaFree(data);
        delete sig;
#endif
        return;
    }

    void SignalProcessor::_compute_2d_fft()
    {
#ifdef WITH_CUDA
        cufftComplex *data;
        size_t fftTotalSize = sizeof(cufftComplex) * this->loops * this->adc_samples * this->virtualAnt;

        float2 *sig = new float2[this->loops * this->adc_samples * this->virtualAnt];

        cudaMalloc((void **)&data, fftTotalSize);
        if (cudaGetLastError() != cudaSuccess)
        {
            std::cerr << "CUDA Error: malloc failed" << std::endl;
        }
#else
#pragma omp parallel
#pragma omp for
#endif
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
#ifndef WITH_CUDA
#pragma omp parallel
#pragma omp for
#endif
            for (int sample = 0; sample < this->adc_samples; ++sample)
            {
                Eigen::MatrixXcd rx_n_range_k = this->_output.fft_1d.block(sample, rx * this->loops,
                                                                           1, this->loops);
                Eigen::dcomplex dc_avg = rx_n_range_k.mean();

                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    /* zero-mean */
                    this->_output.fft_1d(sample, rx * loops + chirp) -= dc_avg;
#ifdef WITH_CUDA
                    int idx = (rx * this->loops * this->adc_samples) +
                              (sample * this->loops) + chirp;
                    sig[idx].x = (float)this->_output.fft_1d(sample, rx * loops + chirp).real();
                    sig[idx].y = (float)this->_output.fft_1d(sample, rx * loops + chirp).imag();
#endif
                }
#ifndef WITH_CUDA
                double fft_container[2 * this->loops];

                // compute DC component

#pragma omp parallel
#pragma omp for
                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    rx_n_range_k(0, chirp) -= dc_avg;
                    fft_container[2 * chirp] = rx_n_range_k(0, chirp).real();
                    fft_container[2 * chirp + 1] = rx_n_range_k(0, chirp).imag();
                }
                /* compute 2D-FFT */
                gsl_fft_complex_radix2_forward(fft_container, 1, this->loops);
                int i = 0;
#pragma omp parallel
#pragma omp for
                for (int i = 0; i < 2 * this->loops; i += 2)
                {
                    Eigen::dcomplex cmplx(fft_container[i], fft_container[i + 1]);
                    this->_output.fft_2d(sample, rx * loops + (i / 2)) = cmplx;
                }
#endif
            }
        }
#ifdef WITH_CUDA
        cudaMemcpy(data, sig, fftTotalSize, cudaMemcpyHostToDevice);

        // run FFT with CUDA
        if (cufftExecC2C(*this->_plan_2d, data, data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }
        if (cudaDeviceSynchronize() != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        cudaMemcpy(sig, data, fftTotalSize, cudaMemcpyDeviceToHost);

        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
            for (int sample = 0; sample < this->adc_samples; ++sample)
            {
                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    int idx = (rx * this->loops * this->adc_samples) + (sample * this->loops) + chirp;
                    Eigen::dcomplex cmplx(sig[idx].x, sig[idx].y);
                    this->_output.fft_2d(sample, rx * loops + chirp) = cmplx;
                }
            }
        }

        cudaFree(data);
        delete sig;
#endif
        return;
    }

    void SignalProcessor::_cfar(Eigen::MatrixXcd &raw_fft)
    {
        Eigen::MatrixXd fft_norm;
        mmfusion::getNormMat(raw_fft, fft_norm);

        this->_output.cfar_1d = Eigen::MatrixXd::Zero(fft_norm.rows(),
                                                      fft_norm.cols());
        int col;
#pragma omp parallel for private(col)
        for (col = 0; col < fft_norm.cols(); ++col)
        {
            Eigen::VectorXd src = fft_norm.col(col);
            this->_output.cfar_1d.col(col) = mmfusion::cfarConv(src);
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