cmake_minimum_required(VERSION 2.8.8)
project(koptercontrol)

add_definitions(-std=c++0x -fPIC)
#ADD_DEFINITIONS(-std=c++0x -fpermissive)

find_package(Qt5Core)

set(CMAKE_AUTOMOC ON)
set(CMAKE_BUILD_TYPE Debug)

include_directories(/usr/include ${CMAKE_CURRENT_SOURCE_DIR}/../common/ ${CMAKE_CURRENT_SOURCE_DIR})

link_directories(/usr/local/lib/)

set(SRC_FILES_KOPTERCONTROL
koptercontrol
koptermessage
kopter
camera
gnssdevice
laserscanner
hokuyo
../common/messagehandler
../common/profiler
../common/gnsstime
../common/logfile
../common/sensorfuser
../common/flightcontroller
../common/flightcontrollervalues
../common/sbfparser
../common/pidcontroller
../common/common
../common/motioncommand
../common/flightstate
../common/flightstaterestriction
../common/pose
../common/rawscan
../common/waypoint
../common/lidarpoint
../common/baseconnection
../common/gnssstatus
../common/vehiclestatus
)

#set(INC_FILES_KOPTERCONTROL
#koptercontrol.h
#koptermessage.h
#kopter.h
#camera.h
#gnssdevice.h
#laserscanner.h
#hokuyo.h
#../common/messagehandler.h
#../common/profiler.h
#../common/logfile.h
#../common/gnsstime.h
#../common/sensorfuser.h
#../common/flightcontroller.h
#../common/flightcontrollervalues.h
#../common/sbfparser.h
#../common/pidcontroller.h
#../common/common.h
#../common/motioncommand.h
#../common/flightstate.h
#../common/flightstaterestriction.h
#../common/pose.h
#../common/rawscan.h
#../common/waypoint.h
#../common/lidarpoint.h
#../common/baseconnection.h
#../common/gnssstatus.h
#../common/vehiclestatus.h
#)

set(URG_LIBRARIES
    urg
    urg_connection
    urg_common
    urg_coordinate
    urg_geometry
    urg_monitor
    urg_system
)

add_executable(koptercontrol ${SRC_FILES_KOPTERCONTROL})
qt5_use_modules(koptercontrol Widgets Gui Core Concurrent Network SerialPort)
target_link_libraries(koptercontrol ${URG_LIBRARIES})
