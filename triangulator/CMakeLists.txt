# The name of our project is "SIMULATOR". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (triangulator)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib64/OGRE/cmake/)

SET(SRC_FILES
main
triangulator
glwidget
logwidget
flightplanner
flightplannerbasic
flightplannerphysics
camerawindow
controlwidget
cloudexporter
../common/lidarpoint
../common/octree
../common/node
../common/common
#../common/coordinateconverter
)

SET(INC_FILES
triangulator.h
glwidget.h
logwidget.h
flightplanner.h
flightplannerbasic.h
flightplannerphysics.h
camerawindow.h
controlwidget.h
cloudexporter.h
../common/lidarpoint.h
../common/octree.h
../common/node.h
../common/common.h
#../common/coordinateconverter.h
)

SET(UI_FILES
controlwidget.ui
logwidget.ui
)

SET(RESOURCE_FILES
)

SET(OGRE_LIBRARIES OgreMain)
SET(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath BulletSoftBody)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

# does not (yet) exist FIND_PACKAGE(BULLET)
FIND_PACKAGE(OGRE)
FIND_PACKAGE(Qt4)
SET(QT_USE_QTGUI 1)
SET(QT_USE_QTXML 1)
SET(QT_USE_QTOPENGL 1)
SET(QT_USE_QTNETWORK 1)
INCLUDE(${QT_USE_FILE})

QT4_WRAP_CPP(MOC_SRCS ${INC_FILES})
QT4_WRAP_UI(UI_SRCS ${UI_FILES})
QT4_ADD_RESOURCES(RESOURCE_SRCS ${RESOURCE_FILES})

SET(CMAKE_DEBUG_POSTFIX "_d")
SET(CMAKE_BUILD_TYPE "Debug")

INCLUDE_DIRECTORIES(/usr/include ${OGRE_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
ADD_EXECUTABLE(triangulator ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
TARGET_LINK_LIBRARIES(triangulator ${QT_LIBRARIES} ${OGRE_LIBRARY_NAMES} ${BULLET_LIBRARIES} GLEW)

#include_directories(/usr/include/OGRE/ /usr/include/bullet /usr/include ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
LINK_DIRECTORIES(/usr/lib/)
#add_executable(triangulator ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
#target_link_libraries(triangulator ${QT_LIBRARIES})
