# The name of our project is "SIMULATOR". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT(basestation)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/cmake/Modules/)

SET(SRC_FILES
    main
    basestation
    glwidget
    logwidget
    logplayer
    ptucontroller
    camerawidget
    waypointlist
    flightplannerinterface
    flightplannerbasic
    flightplannercuda
    flightplannerparticles
    flightplannerphysics
    flightplannerphysicsdialog
    rtkfetcher
    model
    controlwidget
    connectiondialog
    roverconnection
    bulletdebugdrawergl
    voxelmanager
    shaderprogram
    ../common/joystick
    ../common/sbfparser
    ../common/sensorfuser
    ../common/plymanager
    ../common/openglutilities
    ../common/lidarpoint
    ../common/waypoint
    ../common/octree
    ../common/node
    ../common/pose
    ../common/common
    ../common/plotwidget
    ../common/gpsstatusinformation
    ../common/wirelessdevice
    particlesystem
    particlesystem_cuda.cu
    particlerenderer
    #particleskernel.cu
    computefreecolumns.cu
)

SET(INC_FILES
    basestation.h
    glwidget.h
    logwidget.h
    logplayer.h
    ptucontroller.h
    camerawidget.h
    waypointlist.h
    flightplannerinterface.h
    flightplannerbasic.h
    flightplannercuda.h
    flightplannerparticles.h
    flightplannerphysics.h
    flightplannerphysicsdialog.h
    rtkfetcher.h
    model.h
    controlwidget.h
    connectiondialog.h
    roverconnection.h
    bulletdebugdrawergl.h
    voxelmanager.h
    shaderprogram.h
    ../common/joystick.h
    ../common/sbfparser.h
    ../common/sensorfuser.h
    ../common/plymanager.h
    ../common/openglutilities.h
    ../common/lidarpoint.h
    ../common/waypoint.h
    ../common/octree.h
    ../common/node.h
    ../common/pose.h
    ../common/common.h
    ../common/plotwidget.h
    ../common/gpsstatusinformation.h
    ../common/wirelessdevice.h
    particlesystem.h
    particlesystem_cuda.cuh
    particlerenderer.h
    particleskernel.cuh
)

SET(MOC_FILES
    basestation.h
    glwidget.h
    logwidget.h
    logplayer.h
    ptucontroller.h
    camerawidget.h
    flightplannerinterface.h
    flightplannerbasic.h
    flightplannercuda.h
    flightplannerparticles.h
    flightplannerphysics.h
    flightplannerphysicsdialog.h
    rtkfetcher.h
    model.h
    controlwidget.h
    connectiondialog.h
    roverconnection.h
    voxelmanager.h
    shaderprogram.h
    ../common/joystick.h
    ../common/sbfparser.h
    ../common/sensorfuser.h
    ../common/plymanager.h
    ../common/lidarpoint.h
    ../common/octree.h
    ../common/plotwidget.h
    ../common/wirelessdevice.h
    particlesystem.h
    particlerenderer.h
)

SET(UI_FILES
    controlwidget.ui
    logplayer.ui
    ptucontroller.ui
    logwidget.ui
    connectiondialog.ui
    flightplannerphysicsdialog.ui
)

SET(RESOURCE_FILES
)

SET(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath BulletSoftBody)
SET(BULLET_INCLUDES /usr/local/include/bullet/ /usr/include/bullet /usr/include/BulletCollision/BroadphaseCollision/ /usr/include/BulletCollision/CollisionDispatch)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

# does not (yet) exist FIND_PACKAGE(BULLET)
FIND_PACKAGE(Qt4)
SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTOPENGL 1)
SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})

FIND_PACKAGE(CUDA)
FIND_PACKAGE(DevIL)

QT4_WRAP_CPP(MOC_SRCS ${MOC_FILES})
QT4_WRAP_UI(UI_SRCS ${UI_FILES})
QT4_ADD_RESOURCES(RESOURCE_SRCS ${RESOURCE_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

# Enable the Node LidarPoint::node member, in which a lidarpoint remembers in which node it is stored. Increases octree efficiency,
# but also creates a dependency on "Node", which is otherwise unused in koptercontrol project
ADD_DEFINITIONS(-DLIDARPOINT_KEEPS_PARENTNODE -DBASESTATION)
ADD_SUBDIRECTORY(qserialdevice)
INCLUDE_DIRECTORIES(/usr/include
                    /usr/local/include/
                    /usr/local/cuda/computing_sdk/CUDALibraries/common/inc/
                    /opt/cuda-sdk/CUDALibraries/common/inc/
                    ${CMAKE_CURRENT_SOURCE_DIR}/qserialdevice/src/qserialdevice
                    ${BULLET_INCLUDES}
                    ${IL_INCLUDE_DIR}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
LINK_DIRECTORIES(/usr/lib/ /usr/local/lib/)
CUDA_ADD_EXECUTABLE(basestation ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
TARGET_LINK_LIBRARIES(basestation
                      qwt
                      ${QT_LIBRARIES}
                      ${BULLET_LIBRARIES}
                      ${IL_LIBRARIES}
                      GL
                      GLEW
                      qserialdeviced
                      assimp
                      IL)
