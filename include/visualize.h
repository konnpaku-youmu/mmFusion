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
        mmfusion::SystemConf *_cfg;
        
        QApplication *_app;

        MainWindow *_window;

    public:
        DataPlotWrapper(int argc, char **argv, mmfusion::SystemConf &,
                        mmfusion::Radar &, mmfusion::SignalProcessor &);

        ~DataPlotWrapper();

        int show();
    };

} // namespace mmfusion

#endif