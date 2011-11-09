# The name of our project is "lrfreader". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT(koptercontrol)

cmake_policy(SET CMP0015 NEW)

SET(SRC_FILES
koptercontrol
koptermessage
kopter
camera
visualodometry
../common/flightcontroller
../common/common
../common/pose
../common/waypoint
../common/lidarpoint
../common/baseconnection
../common/gpsstatusinformation
gpsdevice
laserscanner
)

SET(INC_FILES
koptercontrol.h
koptermessage.h
kopter.h
camera.h
visualodometry.h
../common/flightcontroller.h
../common/common.h
../common/pose.h
../common/waypoint.h
../common/lidarpoint.h
../common/baseconnection.h
../common/gpsstatusinformation.h
gpsdevice.h
laserscanner.h
)

SET(RESOURCE_FILES
)

SET(URG_LIBRARIES
    urg_connection
    urg_common
    urg_coordinate
    urg_geometry
    #urg_monitor
    urg_system
    urg
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

FIND_PACKAGE(SDL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOC_SRCS ${INC_FILES})
QT4_WRAP_UI(UI_SRCS ${UI_FILES})
QT4_ADD_RESOURCES(RESOURCE_SRCS ${RESOURCE_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

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
ADD_EXECUTABLE(koptercontrol ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
TARGET_LINK_LIBRARIES(koptercontrol ${QT_LIBRARIES} ${URG_LIBRARIES} ${SDLMAIN_LIBRARIES} ${SDL_LIBRARIES} -lqserialdevice -lcv -lhighgui)
