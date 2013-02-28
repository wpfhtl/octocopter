# The name of our project is "SIMULATOR". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.6)
PROJECT(basestation)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/cmake/Modules/)

FILE(GLOB_RECURSE CUDA_SRCS "*.cu")

SET(SRC_FILES
    #${CUDA_SRCS}

    particlesystem.cu
    pointcloudcuda.cu
    grid.cu
    cudahelper.cu
    cudahelper.cpp

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
    #voxelmanager
    shaderprogram
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
    ../common/plotwidget
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
    #pointcloudcuda.cuh
    pointcloudcuda.h


    pointcloudoctree.h
    pointcloudoctreenode.h

    #grid.cuh
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
    #voxelmanager.h
    shaderprogram.h
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
    ../common/plotwidget.h
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

#SET(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath BulletSoftBody)
#SET(BULLET_INCLUDES /usr/local/include/bullet/ /usr/include/bullet /usr/include/BulletCollision/BroadphaseCollision/ /usr/include/BulletCollision/CollisionDispatch)

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


#SET(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE ON)
#SET(BUILD_SHARED_LIBS OFF)
#SET(CUDA_SEPARABLE_COMPILATION ON)
LIST(APPEND CUDA_NVCC_FLAGS -arch=sm_20)
LIST(APPEND CUDA_NVCC_FLAGS --relocatable-device-code=true )
#SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -arch=sm_20")





# Compile the CUDA code to PTX. <my_target> is just a string used to set either the shared library flag <my_target>_EXPORTS and the generated file names' prefixes.
#CUDA_WRAP_SRCS(CUDA_PTX PTX generated_ptx_files ${CUDA_SRCS})

# FindCUDA doesn't look for ptxas, but you can do it yourself:
#find_program(CUDA_PTXAS NAMES ptxas PATHS "${CUDA_TOOLKIT_ROOT_DIR}/bin")

# Now set up the build rules to compile the PTX to CUBINs.
#SET(generated_cubin_files)
#FOREACH(ptx_file ${generated_ptx_files})
    # You can get creative and use things like get_filename_component() to strip off the ptx from the filename.
#    get_filename_component(PTX_NAME ${ptx_file} NAME_WE)
#    set(generated_file "${PTX_NAME}.cubin")
#    add_custom_command(
#        OUTPUT ${generated_file}
        # These output files depend on the source_file and the contents of cmake_dependency_file
#        MAIN_DEPENDENCY "${ptx_file}"
        # Here's the ptxas command
#        COMMAND ${CUDA_PTXAS} --gpu-name compute_20 "${ptx_file}" -o "${generated_file}"
#        COMMENT "Generating ${generated_file}"
#    )

#    LIST(APPEND generated_cubin_files "${generated_file}")
#ENDFOREACH()

#CUDA_ADD_LIBRARY(cudalib
#    ${generated_cubin_files}
#    OPTIONS -DSTUFF="blah blah"
#    RELEASE -DNDEBUG
#    DEBUG -g -DDEBUG
#)

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
    qwt
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

#CUDA_ADD_EXECUTABLE(basestation ${SRC_FILES} main.cpp ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
CUDA_ADD_EXECUTABLE(basestation ${SRC_FILES} main.cpp ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
SET_TARGET_PROPERTIES(basestation PROPERTIES AUTOMOC TRUE)
#ADD_DEPENDENCIES(basestation cudalib)
TARGET_LINK_LIBRARIES(basestation ${LIBRARIES})
