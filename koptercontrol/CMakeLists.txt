# The name of our project is "lrfreader". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT(koptercontrol)

# Starting with gcc-4.6, we might need "-Wl,--no-as-needed in CMAKE_CXX_FLAGS"
SET(CMAKE_CXX_FLAGS "-Wl,--no-as-needed")

cmake_policy(SET CMP0015 NEW)

FIND_PACKAGE(SDL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})

SET(CMAKE_DEBUG_POSTFIX "_d")
#SET(CMAKE_BUILD_TYPE "Debug")

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







### koptercontrol

SET(SRC_FILES_KOPTERCONTROL
koptercontrol
koptermessage
kopter
camera
#visualodometry
sensorfuser
../common/flightcontroller
../common/common
../common/plymanager
../common/pose
../common/waypoint
../common/lidarpoint
../common/baseconnection
../common/gpsstatusinformation
gpsdevice
laserscanner
)

SET(INC_FILES_KOPTERCONTROL
koptercontrol.h
koptermessage.h
kopter.h
camera.h
#visualodometry.h
sensorfuser.h
../common/flightcontroller.h
../common/common.h
../common/plymanager.h
../common/pose.h
../common/waypoint.h
../common/lidarpoint.h
../common/baseconnection.h
../common/gpsstatusinformation.h
gpsdevice.h
laserscanner.h
)


#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES_KOPTERCONTROL})
SOURCE_GROUP("Headers" FILES ${INC_FILES_KOPTERCONTROL})

QT4_WRAP_CPP(MOC_SRCS_KOPTERCONTROL ${INC_FILES_KOPTERCONTROL})


ADD_EXECUTABLE(koptercontrol ${SRC_FILES_KOPTERCONTROL} ${MOC_SRCS_KOPTERCONTROL})
TARGET_LINK_LIBRARIES(koptercontrol ${QT_LIBRARIES} ${URG_LIBRARIES} ${SDLMAIN_LIBRARIES} ${SDL_LIBRARIES} -lqserialdevice)






#### logreplay

SET(SRC_FILES_LOGREPLAY
logreplay
sensorfuser
../common/common
../common/pose
laserscanner
)


SET(INC_FILES_LOGREPLAY
sensorfuser.h
../common/common.h
../common/pose.h
laserscanner.h
)

QT4_WRAP_CPP(MOC_SRCS_LOGREPLAY ${INC_FILES_LOGREPLAY})

ADD_EXECUTABLE(logreplay ${SRC_FILES_LOGREPLAY} ${MOC_SRCS_LOGREPLAY})
TARGET_LINK_LIBRARIES(logreplay ${QT_LIBRARIES} ${URG_LIBRARIES} ${SDLMAIN_LIBRARIES} ${SDL_LIBRARIES} -lqserialdevice)

