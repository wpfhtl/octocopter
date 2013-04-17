# We need 2.8.10 and a FindCUDA from svn, so that separate compilation works.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.10)
PROJECT(basestation)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/cmake/Modules/)

FILE(GLOB_RECURSE CUDA_SRCS "*.cu")

SET(SRC_FILES
    ${CUDA_SRCS}

    cudahelper
    main
    particlesystem
    pointcloudcuda
    basestation
    audioplayer
    glwidget
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
    pointcloudoctree
    pointcloudoctreenode
)

SET(TEST_SRC_FILES
    testmain
    testptu
)

SET(INC_FILES
    cudahelper.h
    pointcloud.h
    pointcloudcuda.h
    pointcloudoctree.h
    pointcloudoctreenode.h
    particlesystem.h
    particlerenderer.h
    basestation.h
    audioplayer.h
    glwidget.h
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

SET(UI_FILES
    controlwidget.ui
    logplayer.ui
    ptucontroller.ui
    logwidget.ui
    connectiondialog.ui
    pidcontrollerwidget.ui
    #flightplannerphysicsdialog.ui
    flightplannerparticlesdialog.ui
)

SET(RESOURCE_FILES
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

FIND_PACKAGE(Qt4)
SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTOPENGL 1)
SET(QT_USE_QTNETWORK 1)
SET(QT_USE_QTTEST 1)
INCLUDE(${QT_USE_FILE})

#FIND_PACKAGE(Bullet)
IF(BULLET_FOUND)
 ADD_DEFINITIONS(-DBASESTATION)
ENDIF(BULLET_FOUND)

FIND_PACKAGE(DevIL)
Find_PACKAGE(SDL)
Find_PACKAGE(SDL_mixer)
FIND_PACKAGE(CUDA)

LIST(APPEND CUDA_NVCC_FLAGS -gencode=arch=compute_30,code=sm_30)
#LIST(APPEND CUDA_NVCC_FLAGS -G -g -gencode=arch=compute_30,code=sm_30)

ADD_DEFINITIONS(-std=c++0x -fpermissive)

QT4_WRAP_UI(UI_SRCS ${UI_FILES})
QT4_ADD_RESOURCES(RESOURCE_SRCS ${RESOURCE_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

# Enable the Node LidarPoint::node member, in which a lidarpoint remembers in which node it is stored. Increases octree efficiency,
# but also creates a dependency on "Node", which is otherwise unused in koptercontrol project
ADD_DEFINITIONS(-DLIDARPOINT_KEEPS_PARENTNODE)
ADD_SUBDIRECTORY(qserialdevice)
INCLUDE_DIRECTORIES(/usr/include
                    /usr/local/include/
                    /usr/local/cuda/computing_sdk/CUDALibraries/common/inc/
                    /opt/cuda-sdk/CUDALibraries/common/inc/
                    ${CMAKE_CURRENT_SOURCE_DIR}/qserialdevice/src/qserialdevice
                    ${BULLET_INCLUDE_DIRS}
                    ${IL_INCLUDE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
LINK_DIRECTORIES(/usr/lib/ /usr/local/lib/)
SET(LIBRARIES
    #qwt
    ${QT_LIBRARIES}
    ${BULLET_LIBRARIES}
    ${IL_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDLMIXER_LIBRARY}
    SDLmain
    GL
    GLEW
    qserialdeviced
    assimp)

SET(CUDA_SEPARABLE_COMPILATION ON)

CUDA_ADD_EXECUTABLE(basestation
    ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS}
    OPTIONS -gencode=arch=compute_30,code=sm_30 # those are the nvcc options only ofr linking
  )
SET_TARGET_PROPERTIES(basestation PROPERTIES AUTOMOC TRUE)
TARGET_LINK_LIBRARIES(basestation ${LIBRARIES})
