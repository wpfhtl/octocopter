# The name of our project is "SIMULATOR". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT(simulator)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib64/OGRE/cmake/ /usr/local/lib/OGRE/cmake/ /usr/share/cmake/Modules/)

SET(SRC_FILES
main
simulator
../common/baseconnection
ogrewidget
battery
../common/flightcontroller
../common/pose
../common/lidarpoint
../common/waypoint
engine
joystick
laserscanner
camera
statuswidget
dialogconfiguration
../common/profiler
../common/coordinategps
vehicle
../common/coordinateconverter
../common/common
BtOgre/BtOgre
)

SET(INC_FILES
simulator.h
../common/baseconnection.h
ogrewidget.h
battery.h
../common/flightcontroller.h
../common/pose.h
../common/lidarpoint.h
../common/waypoint.h
engine.h
joystick.h
laserscanner.h
camera.h
statuswidget.h
dialogconfiguration.h
../common/profiler.h
../common/coordinategps.h
vehicle.h
../common/coordinateconverter.h
../common/common.h
BtOgre/BtOgrePG.h
)

SET(UI_FILES
dialogconfiguration.ui
statuswidget.ui
)

SET(RESOURCE_FILES
)

# use this if bullet is also compiled with double precision
ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION)
SET(BULLET_LIBRARIES BulletDynamics BulletSoftBody BulletCollision LinearMath)
SET(BULLET_INCLUDE_DIRS /usr/local/include/bullet/)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

#FIND_PACKAGE(Bullet)
FIND_PACKAGE(OGRE REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)

SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOC_SRCS ${INC_FILES})
QT4_WRAP_UI(UI_SRCS ${UI_FILES})
QT4_ADD_RESOURCES(RESOURCE_SRCS ${RESOURCE_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug" CACHE STRING "debug oder nicht")

INCLUDE_DIRECTORIES(/usr/include ${BULLET_INCLUDE_DIRS}  ${OGRE_INCLUDE_DIRS} ${OGRE_Terrain_INCLUDE_DIRS} ${OGRE_Paging_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
ADD_EXECUTABLE(simulator ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})

TARGET_LINK_LIBRARIES(simulator ${BULLET_LIBRARIES} ${QT_LIBRARIES} ${OGRE_LIBRARY_NAMES} ${OGRE_Terrain_LIBRARY_NAMES} ${OGRE_RTShaderSystem_LIBRARY_NAMES} ${OGRE_Paging_LIBRARY_NAMES})

ADD_CUSTOM_TARGET(copymedia ALL COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/media ${CMAKE_CURRENT_BINARY_DIR}/media COMMENT "Copying media files to build directory" VERBATIM)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/plugins.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins.cfg)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/plugins_d.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins_d.cfg)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/resources.cfg ${CMAKE_CURRENT_BINARY_DIR}/resources.cfg COPYONLY)
