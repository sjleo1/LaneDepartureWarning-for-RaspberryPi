# CMake for libraries
# /lib/CMakeLists.txt
# Erase the underscore in the name of this file


add_library(devaccs STATIC devaccs.cpp)
add_library(linedetect STATIC linedetect.cpp)

target_include_directories(devaccs PUBLIC ${CMAKE_SOURCE_DIR}/includes)
target_include_directories(linedetect PUBLIC ${CMAKE_SOURCE_DIR}/includes)

target_link_libraries(linedetect devaccs)