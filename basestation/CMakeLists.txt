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

set(SRC_FILES
    ${CUDA_SRCS}
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
    flightplannerinterface
    #flightplannerbasic
    #flightplannercuda
    flightplannerparticles
    #flightplannerphysics
    #flightplannerphysicsdialog
    flightplannerparticlesdialog
    diffcorrfetcher
    model
    controlwidget
    connectiondialog
    roverconnection
    #bulletdebugdrawergl
    shaderprogram
    ../common/profiler
    ../common/motioncommand
    ../common/joystick
    ../common/sbfparser
    ../common/sensorfuser
    ../common/plymanager
    ../common/openglutilities
    ../common/flightcontrollervalues
    ../common/pidcontroller
    ../common/flightstate
    ../common/lidarpoint
    ../common/waypoint
    ../common/gnsstime
    ../common/pose
    ../common/common
    #../common/plotwidget
    ../common/gnssstatus
    ../common/wirelessdevice
    ../common/vehiclestatus
    particlerenderer
    pointcloud
    #pointcloudoctree
    #pointcloudoctreenode
)

set(INC_FILES
    cudahelper.h
    pointcloud.h
    pointcloudcuda.h
    #pointcloudoctree.h
    #pointcloudoctreenode.h
    particlesystem.h
    particlerenderer.h
    basestation.h
    audioplayer.h
    glwindow.h
    logwidget.h
    pidcontrollerwidget.h
    logplayer.h
    ptucontroller.h
    camerawidget.h
    waypointlist.h
    flightplannerinterface.h
    #flightplannerbasic.h
    #flightplannercuda.h
    #flightplannerparticles.h
    #flightplannerphysics.h
    #flightplannerphysicsdialog.h
    flightplannerparticlesdialog.h
    diffcorrfetcher.h
    model.h
    controlwidget.h
    connectiondialog.h
    roverconnection.h
    #bulletdebugdrawergl.h
    shaderprogram.h
    ../common/profiler.h
    ../common/motioncommand.h
    ../common/joystick.h
    ../common/sbfparser.h
    ../common/sensorfuser.h
    ../common/plymanager.h
    ../common/openglutilities.h
    ../common/flightcontrollervalues.h
    ../common/pidcontroller.h
    ../common/flightstate.h
    ../common/lidarpoint.h
    ../common/waypoint.h
    ../common/gnsstime.h
    ../common/pose.h
    ../common/common.h
    #../common/plotwidget.h
    ../common/gnssstatus.h
    ../common/wirelessdevice.h
    ../common/vehiclestatus.h
)

set(UI_FILES
    controlwidget.ui
    logplayer.ui
    ptucontroller.ui
    logwidget.ui
    connectiondialog.ui
    pidcontrollerwidget.ui
    #flightplannerphysicsdialog.ui
    flightplannerparticlesdialog.ui
)

qt5_wrap_ui(UI_SRCS ${UI_FILES})

set(RESOURCE_FILES
)

list(APPEND CUDA_NVCC_FLAGS -gencode=arch=compute_30,code=sm_30)
#list(APPEND CUDA_NVCC_FLAGS -G -g -gencode=arch=compute_30,code=sm_30)

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
target_link_libraries(basestation ${LIBRARIES})
qt5_use_modules(basestation Widgets Gui Core Concurrent Network SerialPort)
