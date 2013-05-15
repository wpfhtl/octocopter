cmake_minimum_required(VERSION 2.8.8)
project(portlink)

add_definitions(-std=c++0x -fPIC)

set(CMAKE_AUTOMOC ON)

find_package(Qt5Core REQUIRED)

set(SRC_FILES
portlink
networkserver
networkclient
serialport
port
)

set(INC_FILES
portlink.h
networkserver.h
networkclient.h
serialport.h
port.h
)

add_executable(portlink ${SRC_FILES})
qt5_use_modules(portlink Core Network SerialPort)
set_target_properties(portlink PROPERTIES AUTOMOC TRUE)
