cmake_minimum_required(VERSION 3.0)
project(mmfusion)
enable_language(CUDA)
add_compile_options(-std=c++17)

set(WITH_GUI ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(CUDA)
find_package(OpenCV 4.5 REQUIRED)
find_package(Boost 1.72 REQUIRED)
find_package(OpenMP QUIET)

if(CUDA_FOUND)
  add_compile_definitions(WITH_CUDA)
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -lcudart -l cufft -l cublas)
  find_library(CUDART_LIBRARY x11 cudart cufft 
               ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})
  set(PROJECT_LIBS 
      -lpthread
      -pthread
      OpenMP::OpenMP_CXX
      ${OpenCV_LIBS} 
      ${CUDA_LIBRARIES}
      ${CUDA_CUFFT_LIBRARIES}
      ${CUDA_CUBLAS_LIBRARIES}
  )
  message("-- Using CUDA")
else()
  find_package(GSL)
  set(PROJECT_LIBS 
      -lpthread
      -pthread
      OpenMP::OpenMP_CXX
      ${OpenCV_LIBS} 
      GSL::gsl
  )
  message("-- CUDA Not Found, fallback to GSL")
endif()

if(${WITH_GUI})
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTORCC ON)
  set(QT_VERSION_MAJOR 5)
  find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
  find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets PrintSupport REQUIRED)
endif()

set(PROJECT_SOURCES src/main.cc 
                    src/cam_calib.cc 
                    src/mmWave.cc 
                    src/signalProc.cc 
                    src/inference.cc 
                    src/utilities.cc
                    src/visualize.cc
                    src/logger.cc
                    thirdparty/qcustomplot.h
                    thirdparty/qcustomplot.cpp
                    src/ui/mainwindow.cc
                    include/ui/mainwindow.h
                    include/ui/mainwindow.ui)

include_directories(
  include
  include/ui
  thirdparty
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CUDA_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_LIBS}
  Qt${QT_VERSION_MAJOR}::Widgets 
  Qt${QT_VERSION_MAJOR}::PrintSupport
)
