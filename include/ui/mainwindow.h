#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "../../src/ui/ui_mainwindow.h"
#include "mmWave.h"
#include "signalProc.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

    void bindDevice(mmfusion::Radar &,
                    mmfusion::SignalProcessor &);

private slots:
    void on_quit_clicked();

    void on_toggleSensor_clicked();

    void on_plot_clicked();

    void refresh_plot();

    void on_antennaBox_currentIndexChanged(int index);

    void on_spinBox_valueChanged(int arg1);

private:
    Ui::MainWindow *ui;

    mmfusion::SignalProcessor *_proc;

    mmfusion::Radar *_radar;

    QVector<double> qv_fft_avg;

    QCPColorMap *colorMap;

    QCPColorScale *colorScale;

    bool _enable_plot;

    uint32_t _frame_cnt;

    int _plot_antenna;

    int _plot_chirp;
};
#endif // MAINWINDOW_H
