#ifndef VIS_H
#define VIS_H

#include <QApplication>

#include "utilities.h"
#include "mmWave.h"
#include "ui/mainwindow.h"

namespace mmfusion
{
    class DataPlot
    {
    private:
        QApplication *_app;

        MainWindow *_window;

        mmfusion::DCA1000 *_data_source;

    public:
        DataPlot(int argc, char **argv, mmfusion::DCA1000 &);

        ~DataPlot();

        int show();
    };

} // namespace mmfusion

#endif