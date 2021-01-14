#include "mainwindow.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->_enable_plot = false;
    this->_plot_antenna = 0;
    this->_plot_chirp = 0;
    this->_frame_cnt = 0;

    // automatically refresh plot every 50ms
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(refresh_plot()));
    timer->start(2);

    // add wave form plotter
    ui->timeDomain->addGraph(); // real
    ui->timeDomain->addGraph(); // imag
    ui->timeDomain->addGraph(); // amplitude
    ui->timeDomain->xAxis->setRange(0, 128);
    ui->timeDomain->yAxis->setRange(-0.5, 0.5);
    ui->timeDomain->xAxis2->setVisible(true);
    ui->timeDomain->xAxis2->setTickLabels(false);
    ui->timeDomain->yAxis2->setVisible(true);
    ui->timeDomain->yAxis2->setTickLabels(false);
    ui->timeDomain->legend->setVisible(true);

    // real part
    QPen real_pen;
    real_pen.setColor(QColor(0, 0, 255, 100));
    real_pen.setStyle(Qt::SolidLine);
    real_pen.setWidth(2);
    ui->timeDomain->graph(0)->setPen(real_pen);
    ui->timeDomain->graph(0)->setLineStyle(QCPGraph::LineStyle::lsLine);
    ui->timeDomain->graph(0)->setScatterStyle(QCPScatterStyle::ssNone);
    ui->timeDomain->graph(0)->setName("In-phase");

    // imag part
    QPen imag_pen;
    imag_pen.setColor(QColor(255, 0, 0, 100));
    imag_pen.setStyle(Qt::DashLine);
    imag_pen.setWidth(2);
    ui->timeDomain->graph(1)->setPen(imag_pen);
    ui->timeDomain->graph(1)->setLineStyle(QCPGraph::LineStyle::lsLine);
    ui->timeDomain->graph(1)->setScatterStyle(QCPScatterStyle::ssNone);
    ui->timeDomain->graph(1)->setName("Quadrature");

    // norm
    QPen norm_pen;
    norm_pen.setColor(QColor(0, 50, 50, 100));
    norm_pen.setStyle(Qt::SolidLine);
    norm_pen.setWidth(3);
    ui->timeDomain->graph(2)->setPen(norm_pen);
    ui->timeDomain->graph(2)->setLineStyle(QCPGraph::LineStyle::lsLine);
    ui->timeDomain->graph(2)->setScatterStyle(QCPScatterStyle::ssNone);
    ui->timeDomain->graph(2)->setName("Norm");

    ui->timeDomain->xAxis->setLabel("Samples");
    ui->timeDomain->yAxis->setLabel("Amplitude (V)");

    // add FFT plotter
    ui->freqDomain->addGraph();
    ui->freqDomain->yAxis->setRange(-2, 25);
    ui->freqDomain->xAxis->setLabel("Frequency");
    ui->freqDomain->yAxis->setLabel("Amplitude");
    ui->freqDomain->xAxis2->setVisible(true);
    ui->freqDomain->xAxis2->setTickLabels(false);
    ui->freqDomain->yAxis2->setVisible(true);
    ui->freqDomain->yAxis2->setTickLabels(false);
    ui->freqDomain->legend->setVisible(true);

    // only plot norm
    QPen fft_pen;
    fft_pen.setColor(QColor(255, 0, 0, 100));
    fft_pen.setStyle(Qt::SolidLine);
    fft_pen.setWidth(2);
    ui->freqDomain->graph(0)->setPen(fft_pen);
    ui->freqDomain->graph(0)->setLineStyle(QCPGraph::LineStyle::lsStepCenter);
    ui->freqDomain->graph(0)->setScatterStyle(QCPScatterStyle::ssNone);
    ui->freqDomain->graph(0)->setName("Norm");

    ui->freqDomain->addGraph();
    ui->freqDomain->graph(1)->setLineStyle(QCPGraph::LineStyle::lsNone);
    ui->freqDomain->graph(1)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui->freqDomain->graph(1)->setName("CFAR");

    // plot spectrogram
    ui->spectro->axisRect()->setupFullAxesBox(true);
    colorMap = new QCPColorMap(ui->spectro->xAxis, ui->spectro->yAxis);
    colorScale = new QCPColorScale(ui->spectro);
    ui->spectro->plotLayout()->addElement(0, 1, colorScale);
    ui->spectro->xAxis->setLabel("Chirps");
    ui->spectro->yAxis->setLabel("Velocity (m/s)");
    ui->spectro->xAxis2->setVisible(true);
    ui->spectro->xAxis2->setTickLabels(false);
    ui->spectro->yAxis2->setVisible(true);
    ui->spectro->yAxis2->setTickLabels(false);
    ui->spectro->legend->setVisible(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::bindDevice(mmfusion::SystemConf &cfg, mmfusion::Radar &dev,
                            mmfusion::SignalProcessor &proc)
{
    this->_cfg = &cfg;
    this->_radar = &dev;
    this->_proc = &proc;
    return;
}

void MainWindow::on_quit_clicked()
{
    this->_radar->sensorStop();
    QApplication::closeAllWindows();
    QApplication::quit();
}

void MainWindow::on_toggleSensor_clicked()
{
    if (ui->toggleSensor->text() == "Stop Sensor")
    {
        this->_radar->sensorStop();
        ui->toggleSensor->setText("Start Sensor");
    }
    else
    {
        this->_radar->sensorStart();
        ui->toggleSensor->setText("Stop Sensor");
    }
}

void MainWindow::on_plot_clicked()
{
    if (ui->plot->text() == "Plot")
    {
        ui->plot->setText("Pause");
        _enable_plot = true;
    }
    else
    {
        ui->plot->setText("Plot");
        _enable_plot = false;
    }
}

void MainWindow::refresh_plot()
{
    if (!_enable_plot)
    {
        return;
    }

    Eigen::MatrixXcd raw_data, fft, _2d_fft;
    Eigen::MatrixXd cfar;

    if (this->_proc->getData(raw_data, fft, _2d_fft, cfar))
    {
        int virtualAnt = this->_cfg->tx_num * this->_cfg->rx_num;
        int loops = raw_data.cols() / virtualAnt;

        Eigen::MatrixXcd rx_n = raw_data.block(0, loops * this->_plot_antenna,
                                               raw_data.rows(), loops);

        Eigen::MatrixXcd fft_n = fft.block(0, loops * this->_plot_antenna,
                                           fft.rows(), loops);

        Eigen::MatrixXd cfar_n = cfar.block(0, loops * this->_plot_antenna,
                                            cfar.rows(), loops);

        Eigen::MatrixXcd fft_2d_n = _2d_fft.block(0, loops * this->_plot_antenna,
                                                  _2d_fft.rows(), loops);

        ui->spinBox->setRange(0, rx_n.cols() - 1);

        QVector<double> qv_x, qv_y_i, qv_y_q, qv_y_n;
        for (size_t row = 0; row < rx_n.rows(); ++row)
        {
            qv_x.append(row);
            qv_y_i.append(rx_n(row, this->_plot_chirp).real());
            qv_y_q.append(rx_n(row, this->_plot_chirp).imag());
            double norm = sqrt(rx_n(row, this->_plot_chirp).real() * rx_n(row, this->_plot_chirp).real() +
                               rx_n(row, this->_plot_chirp).imag() * rx_n(row, this->_plot_chirp).imag());

            qv_y_n.append(norm);
        }
        ui->timeDomain->xAxis->setRange(0, raw_data.rows());
        ui->timeDomain->graph(0)->setData(qv_x, qv_y_i);
        ui->timeDomain->graph(1)->setData(qv_x, qv_y_q);
        ui->timeDomain->graph(2)->setData(qv_x, qv_y_n);
        ui->timeDomain->replot();
        ui->timeDomain->update();

        QVector<double> qv_x_fft, qv_fft;
        for (size_t row = 0; row < fft_n.rows(); ++row)
        {
            qv_x_fft.append(row);
            double norm = sqrt(fft_n(row, this->_plot_chirp).real() * fft_n(row, this->_plot_chirp).real() +
                               fft_n(row, this->_plot_chirp).imag() * fft_n(row, this->_plot_chirp).imag());
            qv_fft.append(norm);
        }

        QVector<double> qv_cfar;
        std::vector<int> active_cell;
        for (size_t row = 0; row < cfar_n.rows(); ++row)
        {
            if (cfar(row, this->_plot_chirp) > 0.5)
            {
                qv_cfar.append(cfar(row, this->_plot_chirp));
                active_cell.push_back(row);
            }
            else
            {
                qv_cfar.append(0);
            }
        }

        ui->freqDomain->xAxis->setRange(0, fft_n.rows());
        ui->freqDomain->graph(0)->setData(qv_x_fft, qv_fft);
        ui->freqDomain->graph(1)->setData(qv_x_fft, qv_cfar);
        ui->freqDomain->replot();
        ui->freqDomain->update();

        colorMap->data()->setSize(512, loops);
        colorMap->data()->setRange(QCPRange(0, 512), QCPRange(-1.01, 1.01));

        Eigen::VectorXcd doppler = Eigen::VectorXcd::Zero(_2d_fft.cols());

        for (auto dist : active_cell)
        {
#pragma omp parallel
#pragma omp for
            for (int col = 0; col < _2d_fft.cols(); ++col)
            {
                doppler(col) += _2d_fft(dist, col);
            }
        }

        Eigen::VectorXcd doppler_n = Eigen::VectorXcd::Zero(loops);
        for (size_t i = loops / 2; i < (3 * loops / 2); ++i)
        {
            doppler_n(i - (loops / 2)) = doppler(i % loops);
        }

        double x, y, z;

        for (int yIndex = 0; yIndex < loops; ++yIndex)
        {
            colorMap->data()->cellToCoord(0, yIndex, &x, &y);
            double norm = log(sqrt(doppler_n(yIndex).real() * doppler_n(yIndex).real() +
                                   doppler_n(yIndex).imag() * doppler_n(yIndex).imag()));
            z = norm;
            colorMap->data()->setCell(this->_frame_cnt, yIndex, z);
        }

        this->_frame_cnt++;

        if (this->_frame_cnt > 511)
        {
            this->_frame_cnt = 0;
        }

        colorScale->setType(QCPAxis::atRight);
        colorMap->setColorScale(colorScale);
        colorMap->setGradient(QCPColorGradient::gpJet);

        ui->spectro->rescaleAxes();
        ui->spectro->replot();
    }

    return;
}

void MainWindow::on_antennaBox_currentIndexChanged(int index)
{
    this->_plot_antenna = index;
    return;
}

void MainWindow::on_spinBox_valueChanged(int arg1)
{
    this->_plot_chirp = arg1;
    return;
}
