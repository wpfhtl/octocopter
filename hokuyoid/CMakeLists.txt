cmake_minimum_required(VERSION 2.8.8)
project(hokuyoid)

include_directories(/usr/local/include)
link_directories(/usr/local/lib/)

set(URG_LIBRARIES
    urg
    urg_connection
    urg_common
    urg_coordinate
    urg_geometry
    urg_monitor
    urg_system
)

add_executable(hokuyoid main)
target_link_libraries(hokuyoid ${URG_LIBRARIES})
