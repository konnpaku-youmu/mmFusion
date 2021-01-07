#include "visualize.h"

namespace mmfusion
{
    DataPlot::DataPlot(int argc, char **argv, mmfusion::DCA1000 &dev)
    {
        this->_app = new QApplication(argc, argv);
        this->_window = new MainWindow();

        this->_window->bindDataSource(dev);
        // this->_data_source = &dev;

        return;
    }

    DataPlot::~DataPlot()
    {
    }

    int DataPlot::show()
    {
        this->_window->show();
        return this->_app->exec();
    }
} // namespace mmfusion