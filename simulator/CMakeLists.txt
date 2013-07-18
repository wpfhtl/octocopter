cmake_minimum_required(VERSION 2.8.10)

project(simulator)

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    /usr/lib64/OGRE/cmake/
    /usr/local/lib/OGRE/cmake/
    /usr/share/cmake/Modules/
    /usr/lib/OGRE/cmake
    /usr/share/OGRE/cmake/modules/
)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OGRE REQUIRED)
find_package(Bullet REQUIRED)
find_package(Boost 1.53.0 COMPONENTS system)

set(SRC_FILES
    main
    simulator
    ogrewidget
    battery
    engine
    laserscanner
    camera
    statuswidget
    dialogconfiguration
    physics
    simulator.rc
    BtOgre/BtOgre
    BtOgre/BtOgrePG.h
    ../common/baseconnection
    ../common/logfile
    ../common/flightstate
    ../common/flightstaterestriction
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
    ../common/joystick
    ../common/profiler
    ../common/coordinategps
    ../common/coordinateconverter
    ../common/common
)

set(INC_FILES
    simulator.h
    ogrewidget.h
    battery.h
    engine.h
    laserscanner.h
    camera.h
    statuswidget.h
    dialogconfiguration.h
    physics.h
    BtOgre/BtOgrePG.h
    BtOgre/BtOgreGP.h
    BtOgre/BtOgreExtras.h
    ../common/baseconnection.h
    ../common/logfile.h
    ../common/flightstate.h
    ../common/flightstaterestriction.h
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
    ../common/joystick.h
    ../common/profiler.h
    ../common/coordinategps.h
    ../common/coordinateconverter.h
    ../common/common.h
)

SET(UI_FILES
    dialogconfiguration.ui
    statuswidget.ui
)

qt5_wrap_ui(UI_SRCS ${UI_FILES})

add_definitions(-std=c++0x -fPIC)

include_directories(
    /usr/include
    ${BULLET_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
    ${OGRE_Terrain_INCLUDE_DIRS}
    ${OGRE_Paging_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/)


set(CMAKE_AUTOMOC_RELAXED_MODE TRUE)
set(CMAKE_AUTOMOC TRUE)
add_executable(simulator ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})

qt5_use_modules(simulator Widgets Gui Core Network OpenGL X11Extras)

target_link_libraries(
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

add_custom_target(copymedia ALL COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/media ${CMAKE_CURRENT_BINARY_DIR}/media COMMENT "Copying media files to build directory" VERBATIM)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/winddata.txt ${CMAKE_CURRENT_BINARY_DIR}/winddata.txt)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/plugins.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins.cfg)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/plugins_d.cfg.in ${CMAKE_CURRENT_BINARY_DIR}/plugins_d.cfg)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/resources.cfg ${CMAKE_CURRENT_BINARY_DIR}/resources.cfg COPYONLY)
