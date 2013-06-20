cmake_minimum_required(VERSION 2.8.8)
project(hokuyoid)

#include_directories(
#    /usr/include
#)

set(SRC_FILES
    main
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

add_executable(hokuyoid ${SRC_FILES})
target_link_libraries(hokuyoid ${URG_LIBRARIES})
