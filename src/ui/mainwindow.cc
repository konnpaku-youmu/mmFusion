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
    ui->timeDomain->yAxis->setRange(-0.15, 0.15);
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
    ui->freqDomain->yAxis->setRange(-1, 2);
    ui->freqDomain->xAxis->setLabel("Range(m)");
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
    ui->freqDomain->graph(1)->setLineStyle(QCPGraph::LineStyle::lsImpulse);
    ui->freqDomain->graph(1)->setScatterStyle(QCPScatterStyle::ssCircle);
    ui->freqDomain->graph(1)->setName("Detected object");

    ui->rv_plot->axisRect()->setupFullAxesBox(true);
    rvMap = new QCPColorMap(ui->rv_plot->xAxis, ui->rv_plot->yAxis);
    rvScale = new QCPColorScale(ui->rv_plot);
    ui->rv_plot->plotLayout()->addElement(0, 1, rvScale);
    ui->rv_plot->xAxis->setLabel("Range (m)");
    ui->rv_plot->yAxis->setLabel("Velocity (m/s)");
    ui->rv_plot->xAxis2->setVisible(true);
    ui->rv_plot->xAxis2->setTickLabels(false);
    ui->rv_plot->yAxis2->setVisible(true);
    ui->rv_plot->yAxis2->setTickLabels(false);
    ui->rv_plot->legend->setVisible(false);

    // plot spectrogram
    ui->spectro->axisRect()->setupFullAxesBox(true);
    colorMap = new QCPColorMap(ui->spectro->xAxis, ui->spectro->yAxis);
    colorScale = new QCPColorScale(ui->spectro);
    ui->spectro->plotLayout()->addElement(0, 1, colorScale);
    ui->spectro->xAxis->setLabel("Frame");
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

    Eigen::MatrixXcf raw_data, fft, _2d_fft;
    Eigen::MatrixXd cfar;

    if (this->_proc->getData(raw_data, fft, _2d_fft, cfar))
    {
        int virtualAnt = this->_cfg->tx_num * this->_cfg->rx_num;
        int loops = raw_data.cols() / virtualAnt;

        Eigen::MatrixXcf rx_n = raw_data.block(0, loops * this->_plot_antenna,
                                               raw_data.rows(), loops);

        Eigen::MatrixXcf fft_n = fft.block(0, loops * this->_plot_antenna,
                                           fft.rows(), loops);

        Eigen::MatrixXd cfar_n = cfar.block(0, loops * this->_plot_antenna,
                                            cfar.rows(), loops);

        Eigen::MatrixXcf fft_2d_n = _2d_fft.block(0, loops * this->_plot_antenna,
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
            qv_x_fft.append(row * 0.095);
            double norm = sqrt(fft_n(row, this->_plot_chirp).real() * fft_n(row, this->_plot_chirp).real() +
                               fft_n(row, this->_plot_chirp).imag() * fft_n(row, this->_plot_chirp).imag());
            qv_fft.append(norm);
        }

        // plot 2dfft
        rvMap->data()->setSize(fft_2d_n.rows(), fft_2d_n.cols());
        rvMap->data()->setRange(QCPRange(0, fft_2d_n.rows()), QCPRange(-1.28, 1.28));

        Eigen::MatrixXd fft_2d_norm, fft_2d_filter;
        mmfusion::getNormMat(fft_2d_n, fft_2d_norm);
        mmfusion::blur2D(fft_2d_norm, fft_2d_filter);

        int max_row;
        double _max_fft = fft_2d_filter.maxCoeff();
        double _fft_mean = fft_2d_filter.mean();
        double _x, _y;
        for (int xIndex = 0; xIndex < fft_2d_norm.rows(); ++xIndex)
        {
            for (int yIndex = (loops / 2); yIndex < (3 * loops / 2); ++yIndex)
            {
                rvMap->data()->cellToCoord(xIndex, yIndex - (loops / 2), &_x, &_y);
                if (fft_2d_filter(xIndex, yIndex % loops) == _max_fft)
                {
                    max_row = xIndex;
                }
                rvMap->data()->setCell(xIndex, yIndex - (loops / 2), (fft_2d_filter(xIndex, yIndex % loops)));
            }
        }

        QVector<double> qv_cfar;
        for (size_t row = 0; row < cfar_n.rows(); ++row)
        {
            if (row == max_row)
            {
                qv_cfar.append(0.8);
            }
            else
            {
                qv_cfar.append(0);
            }
        }

        ui->freqDomain->xAxis->setRange(0, fft_n.rows());
        ui->freqDomain->graph(0)->setData(qv_x_fft, qv_fft);
        ui->freqDomain->graph(1)->setData(qv_x_fft, qv_cfar);
        ui->freqDomain->xAxis->rescale();
        // ui->freqDomain->yAxis->rescale();
        ui->freqDomain->replot();
        ui->freqDomain->update();

        rvScale->setType(QCPAxis::atRight);
        rvMap->setColorScale(rvScale);
        rvMap->setGradient(QCPColorGradient::gpJet);

        ui->rv_plot->rescaleAxes();
        ui->rv_plot->replot();
        ui->rv_plot->update();

        // plot spectrogram
        colorMap->data()->setSize(256, loops);
        colorMap->data()->setRange(QCPRange(0, 256), QCPRange(-1.01, 1.01));

        Eigen::VectorXd doppler = Eigen::VectorXd::Zero(fft_2d_norm.cols());
        for (int row = max_row - 2; row < max_row + 2; ++row)
        {
            doppler += fft_2d_norm.row(row);
        }
        double doppler_avg = doppler.mean();

        Eigen::VectorXd doppler_n = Eigen::VectorXd::Zero(loops);
        for (size_t i = loops / 2; i < (3 * loops / 2); ++i)
        {
            doppler_n(i - (loops / 2)) = doppler(i % loops) - doppler_avg;
        }

        double doppler_min = doppler_n.minCoeff();

        for (size_t i = 0; i < loops; ++i)
        {
            doppler_n(i) -= doppler_min;
        }

        double x, y, z;

        for (int yIndex = 0; yIndex < loops; ++yIndex)
        {
            colorMap->data()->cellToCoord(0, yIndex, &x, &y);
            z = 3.5 * log10(doppler_n(yIndex));
            colorMap->data()->setCell(this->_frame_cnt % 256, yIndex, z);
        }

        this->_frame_cnt++;

        colorScale->setType(QCPAxis::atRight);
        colorMap->setColorScale(colorScale);
        colorMap->setGradient(QCPColorGradient::gpJet);
        colorMap->setDataRange(QCPRange(0, 10));

        ui->spectro->rescaleAxes();
        ui->spectro->replot();
        ui->spectro->update();
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
