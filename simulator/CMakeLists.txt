# The name of our project is "SIMULATOR". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.6)

PROJECT(simulator)

SET(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    /usr/lib64/OGRE/cmake/
    /usr/local/lib/OGRE/cmake/
    /usr/share/cmake/Modules/
    /usr/lib/OGRE/cmake
    /usr/share/OGRE/cmake/modules/
)

SET(SRC_FILES
main
simulator
../common/baseconnection
../common/logfile
ogrewidget
battery
../common/flightstate
../common/flightcontroller
../common/pidcontroller
../common/flightcontrollervalues
../common/gnssstatus
../common/vehiclestatus
../common/gnsstime
../common/pose
../common/motioncommand
../common/lidarpoint
../common/waypoint
engine
../common/joystick
laserscanner
camera
statuswidget
dialogconfiguration
../common/profiler
../common/coordinategps
physics
../common/coordinateconverter
../common/common
BtOgre/BtOgre
BtOgre/BtOgrePG.h
simulator.rc
)

SET(INC_FILES
simulator.h
../common/baseconnection.h
../common/logfile.h
ogrewidget.h
battery.h
../common/flightstate.h
../common/flightcontroller.h
../common/pidcontroller.h
../common/flightcontrollervalues.h
../common/gnssstatus.h
../common/vehiclestatus.h
../common/gnsstime.h
../common/pose.h
../common/motioncommand.h
../common/lidarpoint.h
../common/waypoint.h
engine.h
../common/joystick.h
laserscanner.h
camera.h
statuswidget.h
dialogconfiguration.h
../common/profiler.h
../common/coordinategps.h
physics.h
../common/coordinateconverter.h
../common/common.h
BtOgre/BtOgrePG.h
BtOgre/BtOgreGP.h
BtOgre/BtOgreExtras.h
)

SET(UI_FILES
dialogconfiguration.ui
statuswidget.ui
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

FIND_PACKAGE(OGRE REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(Bullet REQUIRED)
find_package(Boost 1.49.0 COMPONENTS system)

SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})

#QT4_WRAP_CPP(MOC_SRCS ${INC_FILES})
QT4_WRAP_UI(UI_SRCS ${UI_FILES})
QT4_ADD_RESOURCES(RESOURCE_SRCS ${RESOURCE_FILES})

ADD_DEFINITIONS(-std=c++0x)

INCLUDE_DIRECTORIES(
    /usr/include
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE_Terrain_INCLUDE_DIRS}
    ${OGRE_Paging_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/)


SET(CMAKE_AUTOMOC_RELAXED_MODE TRUE)
SET(CMAKE_AUTOMOC TRUE)
ADD_EXECUTABLE(simulator ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})

TARGET_LINK_LIBRARIES(
    simulator
    ${BULLET_LIBRARIES}
    ${Boost_LIBRARIES}
    ${QT_LIBRARIES}
    ${OGRE_LIBRARY_NAMES}
    ${OGRE_Terrain_LIBRARY_NAMES}
    ${OGRE_RTShaderSystem_LIBRARY_NAMES}
    ${OGRE_Paging_LIBRARY_NAMES})

if(UNIX AND NOT APPLE)
    if(NOT DESKTOP_ENTRY)
      set(DESKTOP_ENTRY simulator.desktop)
    endif(NOT DESKTOP_ENTRY)

    add_custom_command(OUTPUT ${DESKTOP_ENTRY}
      COMMAND touch ${DESKTOP_ENTRY}
      COMMAND sh ../create-desktopfile.sh ${CMAKE_INSTALL_PREFIX} >${DESKTOP_ENTRY}
      DEPENDS create-desktopfile.sh
      COMMENT "Generating desktop entry file"
      )
    add_custom_target(DESKTOP_ENTRY_FILE ALL
      DEPENDS ${DESKTOP_ENTRY}
      )

    set( APP_ICON icon128.png )
  endif( UNIX AND NOT APPLE )

ADD_CUSTOM_TARGET(copymedia ALL COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/media ${CMAKE_CURRENT_BINARY_DIR}/media COMMENT "Copying media files to build directory" VERBATIM)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/winddata.txt ${CMAKE_CURRENT_BINARY_DIR}/winddata.txt)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/plugins.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins.cfg)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/plugins_d.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins_d.cfg)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/resources.cfg ${CMAKE_CURRENT_BINARY_DIR}/resources.cfg COPYONLY)
