#ifndef VIS_H
#define VIS_H

#include <QApplication>

#include "utilities.h"
#include "mmWave.h"
#include "signalProc.h"
#include "ui/mainwindow.h"

namespace mmfusion
{
    class DataPlotWrapper
    {
    private:
        QApplication *_app;

        MainWindow *_window;

        mmfusion::DCA1000 *_data_source;

        mmfusion::Radar *_radar;

    public:
        DataPlotWrapper(int argc, char **argv, mmfusion::Radar &,
                        mmfusion::SignalProcessor &);

        ~DataPlotWrapper();

        int show();
    };

} // namespace mmfusion

#endif