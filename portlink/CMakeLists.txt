CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT(portlink)


SET(SRC_FILES
portlink
networkserver
networkclient
serialport
qextserialport/qextserialport
qextserialport/posix_qextserialport
)

SET(INC_FILES
portlink.h
networkserver.h
networkclient.h
serialport.h
port.h
qextserialport/qextserialport.h
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

FIND_PACKAGE(Qt4 COMPONENTS QtCore QtNetwork REQUIRED)

#SET(QT_USE_QTCORE 1)
#SET(QT_USE_QTGUI 0)
#SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})
QT4_WRAP_CPP(MOC_SRCS ${INC_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

ADD_EXECUTABLE(portlink ${SRC_FILES} ${MOC_SRCS})
TARGET_LINK_LIBRARIES(portlink ${QT_LIBRARIES})
