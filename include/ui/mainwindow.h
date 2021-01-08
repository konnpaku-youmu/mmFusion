#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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

    void on_comboBox_currentIndexChanged(const QString &arg1);

    void refresh_plot();

    void on_comboBox_currentIndexChanged(int index);

    void on_spinBox_valueChanged(int arg1);

private:
    Ui::MainWindow *ui;

    mmfusion::SignalProcessor *_proc;

    mmfusion::Radar *_radar;

    bool _enable_plot;

    int _plot_antenna;

    int _plot_chirp;
};
#endif // MAINWINDOW_H
