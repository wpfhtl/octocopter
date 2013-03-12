CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT(filewriter)

SET(SRC_FILES
filewriter
filetowrite
../common/logfile
)

SET(INC_FILES
filewriter.h
filetowrite.h
../common/logfile.h
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

FIND_PACKAGE(Qt4 COMPONENTS QtCore REQUIRED)

ADD_DEFINITIONS(-std=c++0x)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/../common/)

INCLUDE(${QT_USE_FILE})
QT4_WRAP_CPP(MOC_SRCS ${INC_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

ADD_EXECUTABLE(filewriter ${SRC_FILES} ${MOC_SRCS})
TARGET_LINK_LIBRARIES(filewriter ${QT_LIBRARIES})
