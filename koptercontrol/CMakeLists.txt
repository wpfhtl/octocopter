cmake_minimum_required(VERSION 2.8.8)
project(koptercontrol)

add_definitions(-std=c++0x -fPIC)
#ADD_DEFINITIONS(-std=c++0x -fpermissive)

find_package(Qt5Core)

#find_package(DevIL)
#find_package(SDL)
#find_package(SDL_mixer)
#find_package(CUDA)

set(CMAKE_AUTOMOC ON)
set(CMAKE_BUILD_TYPE Debug)

include_directories(
    /usr/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/
)

link_directories(/usr/local/lib/)

set(SRC_FILES_KOPTERCONTROL
koptercontrol
koptermessage
kopter
camera
messagehandler
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
../common/flightstateswitch
../common/pose
../common/waypoint
../common/lidarpoint
../common/baseconnection
../common/gnssstatus
../common/vehiclestatus
gnssdevice
laserscanner
hokuyo
)

set(INC_FILES_KOPTERCONTROL
koptercontrol.h
koptermessage.h
kopter.h
camera.h
messagehandler.h
../common/profiler.h
../common/logfile.h
../common/gnsstime.h
../common/sensorfuser.h
../common/flightcontroller.h
../common/flightcontrollervalues.h
../common/sbfparser.h
../common/pidcontroller.h
../common/common.h
../common/motioncommand.h
../common/flightstate.h
../common/flightstateswitch.h
../common/pose.h
../common/waypoint.h
../common/lidarpoint.h
../common/baseconnection.h
../common/gnssstatus.h
../common/vehiclestatus.h
gnssdevice.h
laserscanner.h
hokuyo.h
)

#"Sources" and "Headers" are the group names in Visual Studio.
source_group("Sources" FILES ${SRC_FILES_KOPTERCONTROL})
source_group("Headers" FILES ${INC_FILES_KOPTERCONTROL})

set(URG_LIBRARIES
    urg_connection
    urg_common
    urg_coordinate
    urg_geometry
    urg_monitor
    urg_system
    urg
)

add_executable(koptercontrol ${SRC_FILES_KOPTERCONTROL})
qt5_use_modules(koptercontrol Widgets Gui Core Concurrent Network SerialPort)
target_link_libraries(koptercontrol ${URG_LIBRARIES})

