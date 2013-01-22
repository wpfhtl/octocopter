CMAKE_MINIMUM_REQUIRED(VERSION 2.8.6)
PROJECT(cloudcompare)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/share/cmake/Modules/)

SET(SRC_FILES
    main
    ../common/plymanager
)

SET(INC_FILES
    ../common/plymanager.h
)

SET(UI_FILES
)

SET(RESOURCE_FILES
)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

# does not (yet) exist FIND_PACKAGE(BULLET)
FIND_PACKAGE(Qt4)
SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTOPENGL 1)
SET(QT_USE_QTNETWORK 1)
SET(QT_USE_QTTEST 1)
INCLUDE(${QT_USE_FILE})

FIND_PACKAGE(CUDA)

ADD_DEFINITIONS(-std=c++0x -fpermissive)

QT4_WRAP_UI(UI_SRCS ${UI_FILES})
QT4_ADD_RESOURCES(RESOURCE_SRCS ${RESOURCE_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

# Enable the Node LidarPoint::node member, in which a lidarpoint remembers in which node it is stored. Increases octree efficiency,
# but also creates a dependency on "Node", which is otherwise unused in koptercontrol project
INCLUDE_DIRECTORIES(/usr/include
                    /usr/local/include/
                    /usr/local/cuda/computing_sdk/CUDALibraries/common/inc/
                    /opt/cuda-sdk/CUDALibraries/common/inc/
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
LINK_DIRECTORIES(/usr/lib/ /usr/local/lib/)
SET(LIBRARIES
    ${QT_LIBRARIES}
)
CUDA_ADD_EXECUTABLE(cloudcompare ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
SET_TARGET_PROPERTIES(cloudcompare PROPERTIES AUTOMOC TRUE)
TARGET_LINK_LIBRARIES(cloudcompare ${LIBRARIES})
