# The name of our project is "lrfreader". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.6)
PROJECT(koptercontrol)

# Starting with gcc-4.6, we might need "-Wl,--no-as-needed" in CMAKE_CXX_FLAGS
# -pg is for using gprof.
SET(CMAKE_CXX_FLAGS "-Wl,--no-as-needed")

cmake_policy(SET CMP0015 NEW)

ADD_DEFINITIONS(-std=c++0x -fpermissive)

FIND_PACKAGE(SDL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

SET(URG_LIBRARIES
    urg_connection
    urg_common
    urg_coordinate
    urg_geometry
    #urg_monitor
    urg_system
    urg
)

INCLUDE_DIRECTORIES(
    /usr/include
    /usr/local/include
    ${SDL_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/qserialdevice/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/
)

LINK_DIRECTORIES(/usr/local/lib/)

### koptercontrol binary

SET(SRC_FILES_KOPTERCONTROL
koptercontrol
koptermessage
kopter
camera
messagehandler
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
)

SET(INC_FILES_KOPTERCONTROL
koptercontrol.h
koptermessage.h
kopter.h
camera.h
messagehandler.h
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
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES_KOPTERCONTROL})
SOURCE_GROUP("Headers" FILES ${INC_FILES_KOPTERCONTROL})

#QT4_WRAP_CPP(MOC_SRCS_KOPTERCONTROL ${INC_FILES_KOPTERCONTROL})

ADD_EXECUTABLE(koptercontrol ${SRC_FILES_KOPTERCONTROL} ${MOC_SRCS})
SET_TARGET_PROPERTIES(koptercontrol PROPERTIES AUTOMOC TRUE)
TARGET_LINK_LIBRARIES(koptercontrol ${QT_LIBRARIES} ${URG_LIBRARIES} ${SDLMAIN_LIBRARIES} ${SDL_LIBRARIES} -lqserialdevice)
