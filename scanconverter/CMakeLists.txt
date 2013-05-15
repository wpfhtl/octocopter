cmake_minimum_required(VERSION 2.8.8)
project(scanconverter)

add_definitions(-std=c++0x -fPIC)

find_package(Qt5Core)

include_directories(
    /usr/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/
)

link_directories(/usr/local/lib/)


set(SRC_FILES_SCANCONVERTER
../common/flightcontrollervalues
../common/common
../common/pidcontroller
../common/motioncommand
../common/flightstate
../common/gnsstime
../common/pose
scanconverter
)

set(INC_FILES_SCANCONVERTER
../common/flightcontrollervalues.h
../common/common.h
../common/pose.h
)

set(CMAKE_AUTOMOC ON)
add_executable(scanconverter ${SRC_FILES_SCANCONVERTER})
qt5_use_modules(scanconverter Core Gui)