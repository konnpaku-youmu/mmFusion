#include "signalProc.h"
namespace mmfusion
{
    SignalProcessor::SignalProcessor(mmfusion::SystemConf &cfg, pthread_mutex_t &mut)
    {
        this->_cfg = &cfg;
        this->_mutex = mut;

        this->virtualAnt = this->_cfg->tx_num * this->_cfg->rx_num;
        this->adc_samples = this->_cfg->adc_samples;
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
        this->_plan = new cufftHandle;
        if (cufftPlan1d(this->_plan, this->adc_samples, CUFFT_C2C, 1) != CUFFT_SUCCESS)
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
        cufftDestroy(*this->_plan);
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
#ifndef WITH_CUDA
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
                Eigen::VectorXcd fft_result = Eigen::VectorXcd::Zero(this->adc_samples);
#ifdef WITH_CUDA
                cufftComplex *data;
                cudaMalloc((void **)&data, sizeof(cufftComplex) * this->adc_samples);
                if (cudaGetLastError() != cudaSuccess)
                {
                    std::cout << "CUDA Error: Failed to allocate memory..." << std::endl;
                }

                float2 *sig = new float2[this->adc_samples];

                for (int sample = 0; sample < this->adc_samples; ++sample)
                {
                    sig[sample].x = (float)this->_output.raw(sample, rx * loops + chirp).real();
                    sig[sample].y = (float)this->_output.raw(sample, rx * loops + chirp).imag();
                }

                cudaMemcpy(data, sig, sizeof(float2) * this->adc_samples, cudaMemcpyHostToDevice);

                // run FFT with CUDA
                if (cufftExecC2C(*this->_plan, data, data, CUFFT_FORWARD) != CUFFT_SUCCESS)
                {
                    std::cerr << "Execute C2C FFT failed..." << std::endl;
                    goto fail_1d;
                }
                if (cudaDeviceSynchronize() != cudaSuccess)
                {
                    std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
                }

                cudaMemcpy(sig, data, sizeof(float2) * this->adc_samples, cudaMemcpyDeviceToHost);

                for (int sample = 0; sample < this->adc_samples; ++sample)
                {
                    Eigen::dcomplex cmplx((double)sig[sample].x,
                                          (double)sig[sample].y);
                    fft_result(sample) = cmplx;
                }

                cudaFree(data);
                delete sig;
#else
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
#endif
                this->_output.fft_1d.col(rx * loops + chirp) = fft_result;
            }
        }
    fail_1d:
        return;
    }

    void SignalProcessor::_compute_2d_fft()
    {
#ifndef WITH_CUDA
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
                Eigen::dcomplex dc_sum = rx_n_range_k.mean();
#ifdef WITH_CUDA
                cufftComplex *data;
                cudaMalloc((void **)&data, sizeof(cufftComplex) * this->loops);
                if (cudaGetLastError() != cudaSuccess)
                {
                    std::cout << "CUDA Error: Failed to allocate memory..." << std::endl;
                }

                float2 *sig = new float2[this->loops];

                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    Eigen::dcomplex cmplx = this->_output.fft_1d(sample, rx * loops + chirp) - dc_sum;
                    
                    sig[chirp].x = (float)cmplx.real();
                    sig[chirp].y = (float)cmplx.imag();
                }

                cudaMemcpy(data, sig, sizeof(float2) * this->loops, cudaMemcpyHostToDevice);

                // run FFT with CUDA
                if (cufftExecC2C(*this->_plan, data, data, CUFFT_FORWARD) != CUFFT_SUCCESS)
                {
                    std::cerr << "Execute C2C FFT failed..." << std::endl;
                    goto fail_2d;
                }
                if (cudaDeviceSynchronize() != cudaSuccess)
                {
                    std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
                }

                cudaMemcpy(sig, data, sizeof(float2) * this->loops, cudaMemcpyDeviceToHost);

                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    Eigen::dcomplex cmplx((double)sig[chirp].x,
                                          (double)sig[chirp].y);
                    this->_output.fft_2d(sample, rx * loops + chirp) = cmplx;
                }

                cudaFree(data);
                delete sig;
#else
                double fft_container[2 * this->loops];

                // compute DC component

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
    fail_2d:
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