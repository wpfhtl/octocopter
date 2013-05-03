CMAKE_MINIMUM_REQUIRED (VERSION 2.8.6)
PROJECT(fclogconverter)

ADD_DEFINITIONS(-std=c++0x -fpermissive)

FIND_PACKAGE(Qt4 REQUIRED)
#SET(QT_USE_QTGUI 1)
INCLUDE(${QT_USE_FILE})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

INCLUDE_DIRECTORIES(
    /usr/include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/
    ${CMAKE_CURRENT_SOURCE_DIR}/../koptercontrol/
)

LINK_DIRECTORIES(/usr/local/lib/)


SET(SRC_FILES_LRFLOGWRITER
../common/flightcontrollervalues
../common/common
../common/pidcontroller
../common/motioncommand
../common/flightstate
../common/gnsstime
../common/pose
fclogconverter
)

SET(INC_FILES_LRFLOGWRITER
../common/flightcontrollervalues.h
../common/common.h
../common/pose.h
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES_LRFLOGWRITER})
SOURCE_GROUP("Headers" FILES ${INC_FILES_LRFLOGWRITER})

#QT4_WRAP_CPP(MOC_SRCS_LRFLOGWRITER ${INC_FILES_LRFLOGWRITER})

ADD_EXECUTABLE(fclogconverter ${SRC_FILES_LRFLOGWRITER} ${MOC_SRCS})
SET_TARGET_PROPERTIES(fclogconverter PROPERTIES AUTOMOC TRUE)
TARGET_LINK_LIBRARIES(fclogconverter ${QT_LIBRARIES})
