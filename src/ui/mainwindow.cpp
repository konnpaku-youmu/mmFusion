#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->_a = 0;
    this->_omega = 0;
    this->_phi = 0;

    ui->timeDomainPlot->setInteraction(QCP::iRangeDrag, true);
    ui->timeDomainPlot->setInteraction(QCP::iRangeZoom, true);
    ui->timeDomainPlot->addGraph();
    ui->timeDomainPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssNone);
    ui->timeDomainPlot->graph(0)->setLineStyle(QCPGraph::LineStyle::lsLine);

    ui->fftPlot->addGraph();

    return;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::bindDataSource(mmfusion::DCA1000 &dev)
{
    this->_data_source = &dev;
    return;
}

void MainWindow::addPoint(double x, double y)
{
    qv_x.append(x);
    qv_y.append(y);
    return;
}

void MainWindow::on_quitButton_clicked()
{
}

void MainWindow::on_plotButton_clicked()
{
    // read raw adc data
    if (this->_data_source->getRawData(_raw_data))
    {
        Eigen::MatrixXcd rx_0 = Eigen::MatrixXcd::Map(this->_raw_data.data(),
                                                      this->_raw_data.rows(),
                                                      this->_raw_data.cols() / 4,
                                                      Eigen::OuterStride<>(4 * this->_raw_data.rows()));
        qv_x.clear();
        qv_y.clear();
        for (size_t row = 0; row < rx_0.rows(); ++row)
        {
            qv_x.append(row);
            qv_y.append(rx_0(row, 0).real() * 100);
        }
    }

    ui->timeDomainPlot->graph(0)->setData(qv_x, qv_y);
    ui->timeDomainPlot->replot();
    ui->timeDomainPlot->update();
    return;
}

void MainWindow::on__A_valueChanged(double arg1)
{
    this->_a = arg1;
    this->_replot();
}

void MainWindow::on__Omega_valueChanged(double arg1)
{
    this->_omega = arg1;
    this->_replot();
}

void MainWindow::on__Phi_valueChanged(double arg1)
{
    this->_phi = arg1;
    this->_replot();
}

void MainWindow::_replot()
{
    qv_x.clear();
    qv_y.clear();
    double range = ui->timeDomainPlot->xAxis->range().upper - ui->timeDomainPlot->xAxis->range().lower;

    for (size_t i = 0; i < 500; i++)
    {
        double x = i * (range / 500) + ui->timeDomainPlot->xAxis->range().lower;
        qv_x.append(x);
        qv_y.append(this->_a * sin(this->_omega * x + this->_phi) + 2);
    }

    ui->timeDomainPlot->graph(0)->setData(qv_x, qv_y);
    ui->timeDomainPlot->replot();
    ui->timeDomainPlot->update();
    return;
}
