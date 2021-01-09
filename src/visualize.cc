#include "visualize.h"

namespace mmfusion
{
    DataPlotWrapper::DataPlotWrapper(int argc, char **argv, mmfusion::Radar &dev0,
                                     mmfusion::SignalProcessor &dev1)
    {
        this->_app = new QApplication(argc, argv);
        this->_window = new MainWindow();

        this->_window->bindDevice(dev0, dev1);

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