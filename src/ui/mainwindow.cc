#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->_enable_plot = false;
    this->_plot_antenna = 0;
    this->_plot_chirp = 0;

    // automatically refresh plot every 50ms
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(refresh_plot()));
    timer->start(50);

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
    ui->freqDomain->yAxis->setRange(-2, 30);
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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::bindDevice(mmfusion::Radar &dev,
                            mmfusion::SignalProcessor &proc)
{
    this->_radar = &dev;
    this->_proc = &proc;
    return;
}

void MainWindow::on_quit_clicked()
{
    this->close();
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

void MainWindow::on_comboBox_currentIndexChanged(const QString &arg1)
{
    if (arg1 == "Rx0")
    {
        this->_plot_antenna = 0;
    }
    else if (arg1 == "Rx1")
    {
        this->_plot_antenna = 1;
    }
    else if (arg1 == "Rx2")
    {
        this->_plot_antenna = 2;
    }
    else if (arg1 == "Rx3")
    {
        this->_plot_antenna = 3;
    }
    return;
}

void MainWindow::refresh_plot()
{
    if (!_enable_plot)
    {
        return;
    }

    Eigen::MatrixXcd raw_data, fft;

    if (this->_proc->getData(raw_data, fft))
    {
        Eigen::MatrixXcd rx_0 = raw_data.block(0, 0, raw_data.rows(), 32);
        
        Eigen::VectorXcd fft0 = fft.col(0);

        ui->spinBox->setRange(0, rx_0.cols() - 1);

        QVector<double> qv_x, qv_y_i, qv_y_q, qv_y_n;
        for (size_t row = 0; row < rx_0.rows(); ++row)
        {
            qv_x.append(row);
            qv_y_i.append(rx_0(row, this->_plot_chirp).real());
            qv_y_q.append(rx_0(row, this->_plot_chirp).imag());
            double norm = sqrt(rx_0(row, this->_plot_chirp).real() * rx_0(row, this->_plot_chirp).real() +
                               rx_0(row, this->_plot_chirp).imag() * rx_0(row, this->_plot_chirp).imag());

            qv_y_n.append(norm);
        }
        ui->timeDomain->xAxis->setRange(0, raw_data.rows());
        ui->timeDomain->graph(0)->setData(qv_x, qv_y_i);
        ui->timeDomain->graph(1)->setData(qv_x, qv_y_q);
        ui->timeDomain->graph(2)->setData(qv_x, qv_y_n);

        QVector<double> qv_x_fft, qv_fft;
        for(size_t row = 0; row < fft0.rows(); ++row)
        {
            qv_x_fft.append(row);
            double norm = sqrt(fft0(row).real() * fft0(row).real() +
                               fft0(row).imag() * fft0(row).imag());
            qv_fft.append(norm);
        }
        ui->freqDomain->xAxis->setRange(0, fft0.rows());
        ui->freqDomain->graph(0)->setData(qv_x_fft, qv_fft);

        ui->timeDomain->replot();
        ui->timeDomain->update();
        ui->freqDomain->replot();
        ui->freqDomain->update();
    }

    return;
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
}

void MainWindow::on_spinBox_valueChanged(int arg1)
{
    this->_plot_chirp = arg1;
    return;
}
