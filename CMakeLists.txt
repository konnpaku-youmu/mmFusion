cmake_minimum_required(VERSION 3.0)
project(mmfusion)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(QT_VERSION_MAJOR 5)

add_compile_options(-std=c++17)

find_package(OpenCV REQUIRED)
find_package(GSL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost 1.72 REQUIRED)
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets PrintSupport REQUIRED)

set(PROJECT_SOURCES src/main.cc 
                    src/cam_calib.cc 
                    src/mmWave.cc 
                    src/signalProc.cc 
                    src/inference.cc 
                    src/utilities.cc
                    src/visualize.cc 
                    src/ui/mainwindow.cc
                    include/ui/mainwindow.h
                    thirdparty/qcustomplot.h
                    thirdparty/qcustomplot.cpp
                    src/ui/mainwindow.ui)

include_directories(
  include
  include/ui
  thirdparty
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_link_libraries(${PROJECT_NAME}
  -lpthread
  -pthread
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
  Qt${QT_VERSION_MAJOR}::Widgets 
  Qt${QT_VERSION_MAJOR}::PrintSupport
  GSL::gsl
)
