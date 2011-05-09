# The name of our project is "SIMULATOR". CMakeLists files in this project can
# refer to the root source directory of the project as ${SIMULATOR_SOURCE_DIR} and
# to the root binary directory of the project as ${SIMULATOR_BINARY_DIR}.
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (basestation)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib64/OGRE/cmake/ /usr/local/lib/OGRE/cmake/ /usr/share/cmake/Modules/)

SET(SRC_FILES
main
basestation
glwidget
logwidget
flightplannerinterface
flightplannerbasic
flightplannerphysics
camerawindow
controlwidget
bulletdebugdrawergl
cloudexporter
../common/openglutilities
../common/lidarpoint
../common/waypoint
../common/octree
../common/node
../common/pose
../common/common
../common/plotwidget
)

SET(INC_FILES
basestation.h
glwidget.h
logwidget.h
flightplannerinterface.h
flightplannerbasic.h
flightplannerphysics.h
camerawindow.h
controlwidget.h
bulletdebugdrawergl.h
cloudexporter.h
../common/openglutilities.h
../common/lidarpoint.h
../common/waypoint.h
../common/octree.h
../common/node.h
../common/pose.h
../common/common.h
../common/plotwidget.h
)

SET(UI_FILES
controlwidget.ui
logwidget.ui
)

SET(RESOURCE_FILES
)

SET(BULLET_LIBRARIES BulletDynamics BulletCollision LinearMath BulletSoftBody)
SET(BULLET_INCLUDES /usr/local/include/bullet/ /usr/include/bullet /usr/include/BulletCollision/BroadphaseCollision/ /usr/include/BulletCollision/CollisionDispatch)

#"Sources" and "Headers" are the group names in Visual Studio.
SOURCE_GROUP("Sources" FILES ${SRC_FILES})
SOURCE_GROUP("Headers" FILES ${INC_FILES})

# does not (yet) exist FIND_PACKAGE(BULLET)
FIND_PACKAGE(OGRE REQUIRED)
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
# use this if bullet is also compiled with double precision
#ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION)

# Enable the Node LidarPoint::node member, in which a lidarpoint remembers in which node it is stored. Increases octree efficiency,
# but also creates a dependency on "Node", which is otherwise unused in koptercontrol project
ADD_DEFINITIONS(-DLIDARPOINT_KEEPS_PARENTNODE)
INCLUDE_DIRECTORIES(/usr/include /usr/include/qwt6 /usr/local/qwt-6.0.0-svn/include/ ${BULLET_INCLUDES} ${OGRE_INCLUDE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../common/)
LINK_DIRECTORIES(/usr/local/qwt-6.0.0-svn/lib/)
ADD_EXECUTABLE(basestation ${SRC_FILES} ${MOC_SRCS} ${UI_SRCS} ${RESOURCE_SRCS})
TARGET_LINK_LIBRARIES(basestation qwt6 ${QT_LIBRARIES} ${OGRE_LIBRARY_NAMES} ${BULLET_LIBRARIES} GLU)
LINK_DIRECTORIES(/usr/lib/)
