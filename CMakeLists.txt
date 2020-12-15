cmake_minimum_required(VERSION 3.0)
project(mmfusion)

add_compile_options(-std=c++17)

find_package(OpenCV REQUIRED)
find_package(Boost 1.72 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cc src/cam_calib.cc src/mmWave.cc
                               src/inference.cc src/utilities.cc)

target_link_libraries(${PROJECT_NAME}
  -lpthread
  -pthread
  ${OpenCV_LIBS}
)
