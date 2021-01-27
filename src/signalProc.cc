#include "signalProc.h"
namespace mmfusion
{
    SignalProcessor::SignalProcessor(mmfusion::SystemConf &cfg, pthread_mutex_t &mut)
    {
        this->_cfg = &cfg;
        this->_mutex = mut;

        this->virtualAnt = this->_cfg->tx_num * this->_cfg->rx_num;
        this->adc_samples = this->_cfg->adc_samples;
        this->loops = this->_cfg->chirp_loops;
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
        this->_plan_3d = new cufftHandle;
        if (cufftPlan1d(this->_plan_1d, this->adc_samples, CUFFT_C2C, this->loops * this->virtualAnt) != CUFFT_SUCCESS ||
            cufftPlan1d(this->_plan_2d, this->loops, CUFFT_C2C, this->adc_samples * this->virtualAnt) != CUFFT_SUCCESS ||
            cufftPlan1d(this->_plan_3d, this->virtualAnt, CUFFT_C2C, this->loops * this->adc_samples) != CUFFT_SUCCESS)
        {
            std::cout << "cuFFT error: Plan creation failed..." << std::endl;
        }
        else
        {
            std::cout << "cuFFT: Plan created" << std::endl;
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
        cufftDestroy(*this->_plan_3d);
#endif
        return;
    }

    void SignalProcessor::bindDevice(mmfusion::DCA1000 &dev)
    {
        this->_capture_board = &dev;
        return;
    }

    /**
     * @brief A method for default signal process with 2D-FFT and CA-CFAR
     * 
     */
    void SignalProcessor::_process()
    {
        this->_output.rw_lock = mmfusion::RWStatus::WRITING;
        if (this->_capture_board->getRawData(this->_output.raw))
        {
            assert(this->loops == this->_output.raw.cols() / virtualAnt);
            assert(this->adc_samples == this->_output.raw.rows());
#ifdef WITH_CUDA
            // compute 1D-FFT
            this->_output.fft_1d = Eigen::MatrixXcd::Zero(this->_output.raw.rows(),
                                                          this->_output.raw.cols());
            this->_compute_1d_fft_cuda();
            this->_cfar(this->_output.fft_1d);

            // compute 2D-FFT
            this->_output.fft_2d = Eigen::MatrixXcd::Zero(this->_output.fft_1d.rows(),
                                                          this->_output.fft_1d.cols());

            this->_compute_2d_fft_cuda();

            // compute 3D-FFT
            this->_output.fft_3d = Eigen::MatrixXcd::Zero(this->_output.fft_2d.rows(),
                                                          this->_output.fft_2d.cols());
            // this->_compute_3d_fft_cuda();
#else
            // compute 1D-FFT
            this->_output.fft_1d = Eigen::MatrixXcd::Zero(this->_output.raw.rows(),
                                                          this->_output.raw.cols());
            this->_compute_1d_fft();
            this->_cfar(this->_output.fft_1d);

            // compute 2D-FFT
            this->_output.fft_2d = Eigen::MatrixXcd::Zero(this->_output.fft_1d.rows(),
                                                          this->_output.fft_1d.cols());

            this->_compute_2d_fft();

            // compute 3D-FFT
            this->_output.fft_3d = Eigen::MatrixXcd::Zero(this->_output.fft_2d.rows(),
                                                          this->_output.fft_2d.cols());
            // this->_compute_3d_fft();
#endif
        }

        this->_output.rw_lock = mmfusion::RWStatus::AVAILABLE;

        return;
    }

#ifdef WITH_CUDA

    void SignalProcessor::_compute_1d_fft_cuda()
    {
        cudaError_t cuda_status;
        cufftComplex *gpu_data;
        uint32_t fftTotalLen = this->virtualAnt * this->loops * this->adc_samples;
        uint32_t fftTotalMemSize = fftTotalLen * sizeof(cufftComplex);

        float2 *signal_holder = new float2[fftTotalLen];

        cuda_status = cudaMalloc((void **)&gpu_data, fftTotalMemSize);
        if (cuda_status != cudaSuccess)
        {
            std::cerr << "CUDA Error: malloc failed with code " << cuda_status << std::endl;
        }

        int rx, chirp, sample;
// #pragma omp parallel for private(rx)
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
// #pragma omp parallel for private(chirp)
            for (chirp = 0; chirp < this->loops; ++chirp)
            {
// #pragma omp parallel for private(sample)
                for (sample = 0; sample < this->adc_samples; ++sample)
                {
                    /* flatten ADC matrix */
                    int idx = (rx * this->loops * this->adc_samples) +
                              (chirp * this->adc_samples) + sample;
                    signal_holder[idx].x = (float)this->_output.raw(sample, rx * loops + chirp).real();
                    signal_holder[idx].y = (float)this->_output.raw(sample, rx * loops + chirp).imag();
                }
            }
        }

        // transfer data from CPU to GPU
        cuda_status = cudaMemcpy(gpu_data, signal_holder, fftTotalMemSize, cudaMemcpyHostToDevice);
        if (cuda_status != cudaSuccess)
        {
            std::cerr << "Data upload failed..." << std::endl;
        }

        // run FFT in-place
        if (cufftExecC2C(*this->_plan_1d, gpu_data, gpu_data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }

        cuda_status = cudaDeviceSynchronize();

        if (cuda_status != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        // download result from GPU
        cuda_status = cudaMemcpy(signal_holder, gpu_data, fftTotalMemSize, cudaMemcpyDeviceToHost);
        if (cuda_status != cudaSuccess)
        {
            std::cerr << "Download result failed..." << std::endl;
        }

        /* assemble 1D-FFT result */
// #pragma omp parallel for private(rx)
        for (rx = 0; rx < this->virtualAnt; ++rx)
        {
// #pragma omp parallel for private(chirp)
            for (chirp = 0; chirp < this->loops; ++chirp)
            {
// #pragma parallel for private(sample)
                for (sample = 0; sample < this->adc_samples; ++sample)
                {
                    int idx = (rx * this->adc_samples * this->loops) +
                              (chirp * this->adc_samples) + sample;
                    Eigen::dcomplex cmplx((double)signal_holder[idx].x, (double)signal_holder[idx].y);
                    this->_output.fft_1d(sample, rx * loops + chirp) = cmplx;
                }
            }
        }

        cudaFree(gpu_data);
        delete signal_holder;

        return;
    }

    void SignalProcessor::_compute_2d_fft_cuda()
    {
        cudaError_t cuda_status;
        cufftComplex *gpu_data;
        uint32_t fftTotalLen = this->virtualAnt * this->adc_samples * this->loops;
        uint32_t fftTotalMemSize = fftTotalLen * sizeof(cufftComplex);

        float2 *signal_holder = new float2[fftTotalLen];

        cuda_status = cudaMalloc((void **)&gpu_data, fftTotalMemSize);
        if (cuda_status != cudaSuccess)
        {
            std::cerr << "CUDA Error: malloc failed with code " << cuda_status << std::endl;
        }

        int rx, range, chirp;
// #pragma omp parallel for private(rx)
        for (rx = 0; rx < this->virtualAnt; ++rx)
        {
// #pragma omp parallel for private(range)
            for (range = 0; range < this->adc_samples; ++range)
            {
                Eigen::MatrixXcd rx_n_range_k = this->_output.fft_1d.block(range, rx * this->loops,
                                                                           1, this->loops);
                // compute DC component
                Eigen::dcomplex dc_avg = rx_n_range_k.mean();

// #pragma omp parallel for private(chirp)
                for (chirp = 0; chirp < this->loops; ++chirp)
                {
                    /* zero-mean */
                    this->_output.fft_1d(range, rx * loops + chirp) -= dc_avg;
                    int idx = (rx * this->loops * this->adc_samples) +
                              (range * this->loops) + chirp;
                    signal_holder[idx].x = (float)this->_output.fft_1d(range, rx * loops + chirp).real();
                    signal_holder[idx].y = (float)this->_output.fft_1d(range, rx * loops + chirp).imag();
                }
            }
        }
        cudaMemcpy(gpu_data, signal_holder, fftTotalMemSize, cudaMemcpyHostToDevice);

        // run FFT with CUDA
        if (cufftExecC2C(*this->_plan_2d, gpu_data, gpu_data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }
        if (cudaDeviceSynchronize() != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        cudaMemcpy(signal_holder, gpu_data, fftTotalLen, cudaMemcpyDeviceToHost);

// #pragma omp parallel for private(rx)
        for (rx = 0; rx < this->virtualAnt; ++rx)
        {
// #pragma omp parallel for private(range)
            for (range = 0; range < this->adc_samples; ++range)
            {
// #pragma omp parallel for private(chirp)
                for (chirp = 0; chirp < this->loops; ++chirp)
                {
                    int idx = (rx * this->loops * this->adc_samples) + (range * this->loops) + chirp;
                    Eigen::dcomplex cmplx(signal_holder[idx].x, signal_holder[idx].y);
                    this->_output.fft_2d(range, rx * loops + chirp) = cmplx;
                }
            }
        }

        cudaFree(gpu_data);
        delete signal_holder;

        return;
    }

    void SignalProcessor::_compute_3d_fft_cuda()
    {
    }

#else
    void SignalProcessor::_compute_1d_fft()
    {
        int rx, chirp, sample;
#pragma omp parallel for private(rx)
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
#pragma omp parallel for private(chirp)
            for (chirp = 0; chirp < this->loops; ++chirp)
            {
                Eigen::VectorXcd fft_result = Eigen::VectorXcd::Zero(this->adc_samples);
                double fft_container[2 * this->adc_samples];

#pragma omp parallel for private(sample)
                for (sample = 0; sample < this->adc_samples; ++sample)
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
            }
        }

        return;
    }

    void SignalProcessor::_compute_2d_fft()
    {
        int rx, range, chirp;
#pragma omp parallel for private(rx)
        for (rx = 0; rx < this->virtualAnt; ++rx)
        {
#pragma omp parallel for private(range)
            for (range = 0; range < this->adc_samples; ++range)
            {
                Eigen::MatrixXcd rx_n_range_k = this->_output.fft_1d.block(range, rx * this->loops,
                                                                           1, this->loops);
                // compute DC component
                Eigen::dcomplex dc_avg = rx_n_range_k.mean();

                double fft_container[2 * this->loops];
#pragma omp parallel for private(chirp)
                for (chirp = 0; chirp < this->loops; ++chirp)
                {
                    rx_n_range_k(0, chirp) -= dc_avg;
                    fft_container[2 * chirp] = rx_n_range_k(0, chirp).real();
                    fft_container[2 * chirp + 1] = rx_n_range_k(0, chirp).imag();
                }

                /* compute 2D-FFT with CPU */
                gsl_fft_complex_radix2_forward(fft_container, 1, this->loops);

                int i = 0;
#pragma omp parallel for private(i)
                for (int i = 0; i < 2 * this->loops; i += 2)
                {
                    Eigen::dcomplex cmplx(fft_container[i], fft_container[i + 1]);
                    this->_output.fft_2d(range, rx * loops + (i / 2)) = cmplx;
                }
            }
        }

        return;
    }

    void SignalProcessor::_compute_3d_fft()
    {
        int sample;
#ifdef WITH_CUDA
        cufftComplex *data;
        size_t fftTotalLen = sizeof(cufftComplex) * this->virtualAnt * this->adc_samples * this->loops;
        float2 *sig = new float2[this->virtualAnt * this->adc_samples * this->loops];

        cudaMalloc((void **)&data, fftTotalLen);
#else
#pragma omp parallel for private(sample)
#endif
        for (sample = 0; sample < this->adc_samples; ++sample)
        {
            int chirp;
#ifndef WITH_CUDA
#pragma omp parallel for private(chirp)
#endif
            for (chirp = 0; chirp < this->loops; ++chirp)
            {
                int rx;
#ifndef WITH_CUDA
                double container[2 * this->virtualAnt];
#pragma omp parallel for private(rx)
#endif
                for (rx = 0; rx < this->virtualAnt; ++rx)
                {
#ifdef WITH_CUDA
                    int idx = (sample * this->loops * this->virtualAnt) + (chirp * this->virtualAnt) + rx;
                    sig[idx].x = (float)this->_output.fft_2d(sample, rx * this->loops + chirp).real();
                    sig[idx].y = (float)this->_output.fft_2d(sample, rx * this->loops + chirp).imag();
#else
                    container[2 * rx] = this->_output.fft_2d(sample, rx * this->loops + chirp).real();
                    container[2 * rx + 1] = this->_output.fft_2d(sample, rx * this->loops + chirp).imag();
#endif
                }
#ifndef WITH_CUDA
                /* Compute 3D-FFT with CPU */
                gsl_fft_complex_radix2_forward(container, 1, this->virtualAnt);
                int i;
#pragma omp parallel for private(i)
                for (i = 0; i < 2 * this->virtualAnt; i += 2)
                {
                    Eigen::dcomplex cmplx(container[i], container[i + 1]);
                    this->_output.fft_3d(sample, (i / 2) * this->loops + chirp) = cmplx;
                }
#endif
            }
        }
#ifdef WITH_CUDA
        cudaMemcpy(data, sig, fftTotalLen, cudaMemcpyHostToDevice);

        if (cufftExecC2C(*this->_plan_3d, data, data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }
        if (cudaDeviceSynchronize() != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        cudaMemcpy(sig, data, fftTotalLen, cudaMemcpyDeviceToHost);

#pragma omp parallel for private(sample)
        for (sample = 0; sample < this->adc_samples; ++sample)
        {
            int chirp;
#pragma omp parallel for private(chirp)
            for (chirp = 0; chirp < this->loops; ++chirp)
            {
                int rx;
#pragma omp parallel for private(rx)
                for (rx = 0; rx < this->virtualAnt; ++rx)
                {
                    int idx = (sample * this->loops * this->virtualAnt) +
                              (chirp * this->virtualAnt) + rx;
                    Eigen::dcomplex cmplx(sig[idx].x, sig[idx].y);
                    this->_output.fft_3d(sample, rx * loops + chirp) = cmplx;
                }
            }
        }

        cudaFree(data);
        delete sig;
#endif
        return;
    }
#endif

    void SignalProcessor::_beamforming()
    {
        int N_angle = 8;
        double range = M_PI_2;
        Eigen::MatrixXcd steering_mat = Eigen::MatrixXcd::Zero(this->virtualAnt, N_angle);
        int n;
#pragma omp parallel for private(n)
        for (n = 0; n < N_angle; ++n)
        {
            double theta = ((2 * range) / N_angle) - range;
            double tao = cos(theta) / 2;
            int rx;
#pragma omp parallel for private(rx)
            for (rx = 0; rx < this->virtualAnt; ++rx)
            {
                double shift = 2 * M_PI * tao * (rx - 1);
                steering_mat(rx, n) = Eigen::dcomplex(cos(shift), sin(shift));
            }
        }

#ifdef WITH_CUDA
        cudaError_t cudaStatus;
        cublasStatus_t cublasStatus;
        cublasHandle_t handle;
#endif

        int sample;
#pragma omp parallel for private(sample)
        for (sample = 0; sample < this->adc_samples; ++sample)
        {
            int chirp;
#pragma omp parallel for private(chirp)
            for (chirp = 0; chirp < this->loops; ++chirp)
            {
                Eigen::VectorXcd x_t = Eigen::VectorXcd::Zero(this->virtualAnt);
                int rx;
#pragma omp parallel for private(rx)
                for (rx = 0; rx < this->virtualAnt; ++rx)
                {
                    x_t(rx) = this->_output.raw(sample, rx * loops + chirp);
                }
#ifdef WITH_CUDA

#else
                Eigen::MatrixXcd cov = Eigen::MatrixXcd::Zero(this->virtualAnt, this->virtualAnt);
                cov = x_t * x_t.transpose().conjugate();
                Eigen::VectorXcd power = (steering_mat.conjugate().transpose() * cov * steering_mat).diagonal();
#endif
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