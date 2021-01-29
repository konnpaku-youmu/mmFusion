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

            // compute 1D-FFT
            this->_output.fft_1d = Eigen::MatrixXcf::Zero(this->_output.raw.rows(),
                                                          this->_output.raw.cols());
            this->_compute_1d_fft();
            this->_cfar(this->_output.fft_1d);

            // compute 2D-FFT
            this->_output.fft_2d = Eigen::MatrixXcf::Zero(this->_output.fft_1d.rows(),
                                                          this->_output.fft_1d.cols());

            this->_compute_2d_fft();

            // compute 3D-FFT
            this->_output.fft_3d = Eigen::MatrixXcf::Zero(this->_output.fft_2d.rows(),
                                                          this->_output.fft_2d.cols());
            this->_compute_3d_fft();
        }

        this->_output.rw_lock = mmfusion::RWStatus::AVAILABLE;

        return;
    }

    void SignalProcessor::_compute_1d_fft()
    {
#ifdef WITH_CUDA
        cudaError_t cuda_status;
        cufftComplex *gpu_data;
        size_t fftTotalLen = this->loops * this->virtualAnt * this->adc_samples;
        size_t fftTotalMemSize = sizeof(cufftComplex) * fftTotalLen;

        cuda_status = cudaMalloc((void **)&gpu_data, fftTotalMemSize);
        if (cuda_status != cudaSuccess)
        {
            std::cerr << "CUDA Error: malloc..." << std::endl;
        }

        cuda_status = cudaMemcpy(gpu_data, this->_output.raw.data(), fftTotalMemSize, cudaMemcpyHostToDevice);

        // run FFT with CUDA
        if (cufftExecC2C(*this->_plan_1d, gpu_data, gpu_data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }
        if (cudaDeviceSynchronize() != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        cudaMemcpy(this->_output.fft_1d.data(), gpu_data, fftTotalMemSize, cudaMemcpyDeviceToHost);

        cudaFree(gpu_data);
#else
#pragma omp parallel
#pragma omp for
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
#pragma omp parallel
#pragma omp for
            for (int chirp = 0; chirp < this->loops; ++chirp)
            {
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
            }
        }
#endif
        return;
    }

    void SignalProcessor::_compute_2d_fft()
    {
#ifdef WITH_CUDA
        cufftComplex *data;
        size_t fftTotalLen = this->loops * this->adc_samples * this->virtualAnt;
        size_t fftTotalMemSize = sizeof(cufftComplex) * fftTotalLen;

        cudaMalloc((void **)&data, fftTotalMemSize);
        if (cudaGetLastError() != cudaSuccess)
        {
            std::cerr << "CUDA Error: malloc failed" << std::endl;
        }

        for (int sample = 0; sample < this->adc_samples; ++sample)
        {
            for (int rx = 0; rx < this->virtualAnt; ++rx)
            {
                Eigen::MatrixXcf rx_n_range_k = this->_output.fft_1d.block(sample, rx * this->loops,
                                                                           1, this->loops);
                Eigen::scomplex dc_avg = rx_n_range_k.mean();

                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    /* zero-mean */
                    this->_output.fft_1d(sample, rx * loops + chirp) -= dc_avg;
                }
            }
        }

        Eigen::MatrixXcf fft_1d_transpose = this->_output.fft_1d.transpose();

        cudaMemcpy(data, fft_1d_transpose.data(), fftTotalMemSize, cudaMemcpyHostToDevice);

        // run FFT with CUDA
        if (cufftExecC2C(*this->_plan_2d, data, data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }
        if (cudaDeviceSynchronize() != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        Eigen::MatrixXcf fft_2d_transpose = Eigen::MatrixXcf::Zero(this->_output.fft_2d.cols(),
                                                                   this->_output.fft_2d.rows());
        cudaMemcpy(fft_2d_transpose.data(), data, fftTotalMemSize, cudaMemcpyDeviceToHost);

        this->_output.fft_2d = fft_2d_transpose.transpose();

        cudaFree(data);
#else
#pragma omp parallel
#pragma omp for
        for (int rx = 0; rx < this->virtualAnt; ++rx)
        {
#pragma omp parallel
#pragma omp for
            for (int sample = 0; sample < this->adc_samples; ++sample)
            {
                Eigen::MatrixXcf rx_n_range_k = this->_output.fft_1d.block(sample, rx * this->loops,
                                                                           1, this->loops);
                Eigen::scomplex dc_avg = rx_n_range_k.mean();

                for (int chirp = 0; chirp < this->loops; ++chirp)
                {
                    /* zero-mean */
                    this->_output.fft_1d(sample, rx * loops + chirp) -= dc_avg;
                }
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
                    Eigen::scomplex cmplx(fft_container[i], fft_container[i + 1]);
                    this->_output.fft_2d(sample, rx * loops + (i / 2)) = cmplx;
                }
            }
        }
#endif
        return;
    }

    void SignalProcessor::_compute_3d_fft()
    {
        int sample;
#ifdef WITH_CUDA
        cudaError_t cuda_status;
        cufftComplex *data;
        size_t fftTotalLen = this->virtualAnt * this->adc_samples * this->loops;
        size_t fftTotalMemSize = sizeof(cufftComplex) * fftTotalLen;

        cudaMalloc((void **)&data, fftTotalMemSize);

        Eigen::Map<Eigen::MatrixXcf> fft_2d_reshape(this->_output.fft_2d.data(),
                                                    this->adc_samples * this->loops,
                                                    this->virtualAnt);

        Eigen::MatrixXcf fft_2d_reshape_transpose = fft_2d_reshape.transpose();

        cudaMemcpy(data, fft_2d_reshape_transpose.data(), fftTotalMemSize, cudaMemcpyHostToDevice);

        if (cufftExecC2C(*this->_plan_3d, data, data, CUFFT_FORWARD) != CUFFT_SUCCESS)
        {
            std::cerr << "Execute C2C FFT failed..." << std::endl;
        }
        if (cudaDeviceSynchronize() != cudaSuccess)
        {
            std::cerr << "CUDA Error: Failed to synchronize..." << std::endl;
        }

        Eigen::MatrixXcf fft_3d_reshaped_transpose = Eigen::MatrixXcf::Zero(fft_2d_reshape_transpose.rows(),
                                                                            fft_2d_reshape_transpose.cols());

        cudaMemcpy(fft_3d_reshaped_transpose.data(), data, fftTotalMemSize, cudaMemcpyDeviceToHost);

        Eigen::MatrixXcf fft_3d_reshaped = fft_3d_reshaped_transpose.transpose();

        this->_output.fft_3d = Eigen::Map<Eigen::MatrixXcf>(fft_3d_reshaped.data(),
                                                            this->_output.fft_3d.rows(),
                                                            this->_output.fft_3d.cols());

        cudaFree(data);
#else
#pragma omp parallel for private(sample)
        for (sample = 0; sample < this->adc_samples; ++sample)
        {
            int chirp;
#pragma omp parallel for private(chirp)
            for (chirp = 0; chirp < this->loops; ++chirp)
            {
                int rx;
                double container[2 * this->virtualAnt];
#pragma omp parallel for private(rx)
                for (rx = 0; rx < this->virtualAnt; ++rx)
                {
                    container[2 * rx] = this->_output.fft_2d(sample, rx * this->loops + chirp).real();
                    container[2 * rx + 1] = this->_output.fft_2d(sample, rx * this->loops + chirp).imag();
                }
                /* Compute 3D-FFT with CPU */
                gsl_fft_complex_radix2_forward(container, 1, this->virtualAnt);
                int i;
#pragma omp parallel for private(i)
                for (i = 0; i < 2 * this->virtualAnt; i += 2)
                {
                    Eigen::scomplex cmplx(container[i], container[i + 1]);
                    this->_output.fft_3d(sample, (i / 2) * this->loops + chirp) = cmplx;
                }
            }
        }
#endif
        return;
    }

    void SignalProcessor::_cfar(Eigen::MatrixXcf &raw_fft)
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

    bool SignalProcessor::getData(Eigen::MatrixXcf &raw_adc,
                                  Eigen::MatrixXcf &fft1,
                                  Eigen::MatrixXcf &fft2,
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