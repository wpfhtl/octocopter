# The name of our project is "SIMULATOR". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT(simulator)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib64/OGRE/cmake/)

SET(SRC_FILES
main
simulator
ogrewidget
battery
flightcontroller
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
BtOgre/BtOgre
)

SET(INC_FILES
simulator.h
ogrewidget.h
battery.h
flightcontroller.h
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
BtOgre/BtOgrePG.h
)

SET(UI_FILES
dialogconfiguration.ui
statuswidget.ui
)

SET(RESOURCE_FILES
)

SET(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath BulletSoftBody)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

# does not (yet) exist FIND_PACKAGE(BULLET)
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
SET(CMAKE_BUILD_TYPE "Debug")

#/usr/local/include/bullet
INCLUDE_DIRECTORIES(/usr/include ${OGRE_INCLUDE_DIR} ${OGRE_Terrain_INCLUDE_DIRS} ${OGRE_Paging_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
ADD_EXECUTABLE(simulator ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
TARGET_LINK_LIBRARIES(simulator ${QT_LIBRARIES} ${OGRE_LIBRARY_NAMES} ${OGRE_Terrain_LIBRARY_NAMES} ${OGRE_RTShaderSystem_LIBRARY_NAMES} ${BULLET_LIBRARIES})

ADD_CUSTOM_TARGET(copymedia ALL COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/media ${CMAKE_CURRENT_BINARY_DIR}/media COMMENT "Copying media files to build directory" VERBATIM)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/plugins.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins.cfg)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/plugins_d.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins_d.cfg)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/resources.cfg ${CMAKE_CURRENT_BINARY_DIR}/resources.cfg COPYONLY)
