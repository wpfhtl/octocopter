cmake_minimum_required(VERSION 2.8.8)
project(lrfreader)

add_definitions(-std=c++0x -fPIC)

find_package(Qt5Core)

include_directories(
    /usr/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../common/
)

set(SRC_FILES
    main
    ../koptercontrol/hokuyo
    ../koptercontrol/laserscanner
    ../common/logfile
    ../common/pose
    ../common/gnsstime
)

set(INC_FILES
    ../koptercontrol/hokuyo.h
    ../koptercontrol/laserscanner.h
    ../common/logfile.h
    ../common/pose.h
    ../common/gnsstime.h
)

set(URG_LIBRARIES
    urg_connection
    urg_common
    urg_coordinate
    urg_geometry
    urg_monitor
    urg_system
    urg
c_urg
c_urg_system
c_urg_connection
)

add_executable(lrfreader ${SRC_FILES})
target_link_libraries(lrfreader ${URG_LIBRARIES})
qt5_use_modules(lrfreader Core Gui)
set_target_properties(lrfreader PROPERTIES AUTOMOC TRUE)
