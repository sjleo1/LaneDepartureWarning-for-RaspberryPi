cmake_minimum_required (VERSION 2.8)

# OpenCV, raspicam, ALSA are required to install prior to build this program

project(ADAS-RPi LANGUAGES CXX)

set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/${CMAKE_MODULE_PATH}")

find_package(raspicam REQUIRED)
find_package(OpenCV)
find_package(ALSA REQUIRED)

add_subdirectory(lib)

add_executable(adas adas_main.cpp)

target_link_libraries(adas ${raspicam_CV_LIBS} ${raspicam_LIBS})
target_link_libraries(adas ${ALSA_LIBRARIES})
target_link_libraries(adas devaccs)
target_link_libraries(adas linedetect)

set_target_properties(adas PROPERTIES LINKER_LANGUAGES CXX)