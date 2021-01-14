#include "visualize.h"

namespace mmfusion
{
    DataPlotWrapper::DataPlotWrapper(int argc, char **argv,
                                     mmfusion::SystemConf &cfg,
                                     mmfusion::Radar &radar,
                                     mmfusion::SignalProcessor &proc)
    {
        this->_cfg = &cfg;
        this->_app = new QApplication(argc, argv);
        this->_window = new MainWindow();

        this->_window->bindDevice(radar, proc);

        return;
    }

    DataPlotWrapper::~DataPlotWrapper()
    {
    }

    int DataPlotWrapper::show()
    {
        this->_window->show();
        return this->_app->exec();
    }
} // namespace mmfusion