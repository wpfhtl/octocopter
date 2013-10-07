# We need 2.8.10 and a FindCUDA from svn, so that separate compilation works.
cmake_minimum_required(VERSION 2.8.10)
project(basestation)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/cmake/Modules/)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED) # !!!
find_package(Qt5Widgets REQUIRED)
find_package(DevIL REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_mixer REQUIRED)
find_package(CUDA REQUIRED)

file(GLOB_RECURSE CUDA_SRCS "*.cu")
file(GLOB_RECURSE NORAD_SRCS "../common/sgp4sdp4/*.c")

set(SRC_FILES
    ${CUDA_SRCS}
    ${NORAD_SRCS}
    cudahelper
    main
    particlesystem
    pointcloudcuda
    basestation
    audioplayer
    glwindow
    logwidget
    pidcontrollerwidget
    logplayer
    ptucontroller
    camerawidget
    waypointlist
    pathplanner
    flightplannerparticles
    flightplannerparticlesdialog
    diffcorrfetcher
    model
    controlwidget
    satellitewidget
    connectiondialog
    roverconnection
    shaderprogram
    ../common/messagehandler
    ../common/profiler
    ../common/rawscan
    ../common/logfile
    ../common/motioncommand
    ../common/joystick
    ../common/sbfparser
    ../common/sensorfuser
    ../common/plymanager
    ../common/openglutilities
    ../common/flightcontrollervalues
    ../common/pidcontroller
    ../common/flightstate
    ../common/flightstaterestriction
    ../common/lidarpoint
    ../common/waypoint
    ../common/gnsstime
    ../common/pose
    ../common/box3d
    ../common/common
    ../common/gnssstatus
    ../common/wirelessdevice
    ../common/vehiclestatus
    ../common/satellite
    ../common/almanac
    ../common/tsp/fasttsp
    ../common/tsp/graph
    ../common/tsp/path
    ../common/tsp/opttsp
    ../common/tsp/pathinfo
    glscene
    pointcloud
)

set(UI_FILES
    controlwidget.ui
    logplayer.ui
    ptucontroller.ui
    logwidget.ui
    connectiondialog.ui
    pidcontrollerwidget.ui
    satellitewidget.ui
    flightplannerparticlesdialog.ui
)

qt5_wrap_ui(UI_SRCS ${UI_FILES})

set(RESOURCE_FILES
)

execute_process(COMMAND glxinfo COMMAND grep "OpenGL version string" COMMAND cut -d " " -f 4 COMMAND cut -d "." -f 1 OUTPUT_VARIABLE openglversion_major OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND glxinfo COMMAND grep "OpenGL version string" COMMAND cut -d " " -f 4 COMMAND cut -d "." -f 2 OUTPUT_VARIABLE openglversion_minor OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Using OpenGl version \"${openglversion_major}.${openglversion_minor}\" and class \"QOpenGLFunctions_${openglversion_major}_${openglversion_minor}_Core\"")
add_definitions(-DOPENGL_FUNCTIONS_CLASS=QOpenGLFunctions_${openglversion_major}_${openglversion_minor}_Core -DOPENGL_VERSION_MAJOR=${openglversion_major} -DOPENGL_VERSION_MINOR=${openglversion_minor})

# nvcc options for debugging:
#--debug (-g)                        Generate debug information for host code.
#--device-debug (-G)                 Generate debug information for device code.
#--generate-line-info (-lineinfo)    Generate line-number information for device code.
list(APPEND CUDA_NVCC_FLAGS -gencode=arch=compute_30,code=sm_30)
#list(APPEND CUDA_NVCC_FLAGS -gencode=arch=compute_30,code=sm_30 -g -G -lineinfo)

add_definitions(-std=c++0x -fPIC)

set(CMAKE_BUILD_TYPE Debug)

include_directories(/usr/include
                    /usr/local/cuda/computing_sdk/CUDALibraries/common/inc/
                    ${Qt5Gui_INCLUDES} # !!!
                    ${IL_INCLUDE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../common/)

link_directories(/usr/lib/ /usr/local/lib/)

set(LIBRARIES
    ${IL_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDLMIXER_LIBRARY}
    SDLmain
    assimp)

set(CUDA_SEPARABLE_COMPILATION ON)
set(CMAKE_AUTOMOC ON)

cuda_add_executable(basestation ${SRC_FILES} ${UI_SRCS} ${RESOURCE_SRCS} OPTIONS -gencode=arch=compute_30,code=sm_30) # those are the nvcc options only for linking
qt5_use_modules(basestation Widgets Gui Core Concurrent Network SerialPort OpenGL)
target_link_libraries(basestation ${LIBRARIES})

cuda_add_executable(test_pcd_reduction
    ${CUDA_SRCS}
    tests/testpcdreduction
    cudahelper
    pointcloudcuda
    ../common/profiler
    ../common/plymanager
    ../common/box3d
    ../common/common
    pointcloud
    OPTIONS -gencode=arch=compute_30,code=sm_30)

qt5_use_modules(test_pcd_reduction Widgets Core Concurrent Network SerialPort)
target_link_libraries(test_pcd_reduction ${LIBRARIES})
