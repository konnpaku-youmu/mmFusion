#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "mmWave.h"

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();
    
    void bindDataSource(mmfusion::DCA1000 &);

    void addPoint(double x, double y);

private slots:
    void on_quitButton_clicked();

    void on_plotButton_clicked();

    void on__A_valueChanged(double arg1);

    void on__Omega_valueChanged(double arg1);

    void on__Phi_valueChanged(double arg1);

private:
    Ui::MainWindow *ui;

    Eigen::MatrixXcd _raw_data;

    mmfusion::DCA1000 *_data_source;

    QVector<double> qv_x, qv_y;

    double _a, _omega, _phi;

    void _replot();
};
#endif // MAINWINDOW_H
