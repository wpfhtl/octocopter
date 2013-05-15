cmake_minimum_required(VERSION 2.8.8)
project(filewriter)

add_definitions(-std=c++0x -fPIC)

find_package(Qt5Core)

include_directories(
    /usr/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/
)

link_directories(/usr/local/lib/)

set(SRC_FILES
filewriter
filetowrite
../common/logfile
)

set(INC_FILES
filewriter.h
filetowrite.h
../common/logfile.h
)

set(CMAKE_AUTOMOC ON)
add_executable(filewriter ${SRC_FILES})
qt5_use_modules(filewriter Core)