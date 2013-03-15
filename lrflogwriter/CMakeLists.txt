# The name of our project is "lrfreader". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.6)
PROJECT(lrflogwriter)

# Starting with gcc-4.6, we might need "-Wl,--no-as-needed" in CMAKE_CXX_FLAGS
# -pg is for using gprof.
SET(CMAKE_CXX_FLAGS "-Wl,--no-as-needed")

cmake_policy(SET CMP0015 NEW)

ADD_DEFINITIONS(-std=c++0x -fpermissive)

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
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/
    ${CMAKE_CURRENT_SOURCE_DIR}/../koptercontrol/
)

LINK_DIRECTORIES(/usr/local/lib/)

### lrflogwriter binary

SET(SRC_FILES_LRFLOGWRITER
../common/logfile
../common/common
../common/gnsstime
../common/pose
../koptercontrol/laserscanner
lrflogwriter
)

SET(INC_FILES_LRFLOGWRITER
../common/logfile.h
../common/common.h
../common/gnsstime.h
../common/pose.h
../koptercontrol/laserscanner.h
lrflogwriter.h
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES_LRFLOGWRITER})
SOURCE_GROUP("Headers" FILES ${INC_FILES_LRFLOGWRITER})

#QT4_WRAP_CPP(MOC_SRCS_LRFLOGWRITER ${INC_FILES_LRFLOGWRITER})

ADD_EXECUTABLE(lrflogwriter ${SRC_FILES_LRFLOGWRITER} ${MOC_SRCS})
SET_TARGET_PROPERTIES(lrflogwriter PROPERTIES AUTOMOC TRUE)
TARGET_LINK_LIBRARIES(lrflogwriter ${QT_LIBRARIES} ${URG_LIBRARIES} ${SDLMAIN_LIBRARIES} ${SDL_LIBRARIES} -lqserialdevice)
