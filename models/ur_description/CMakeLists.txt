CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
INCLUDE(cmake/base.cmake)
#include(cmake/ros.cmake)


SET(PROJECT_NAME ur5_description)
SET(PROJECT_DESCRIPTION "UR5 urdf model, srdf model and other data")
SET(PROJECT_URL "d")

set(CXX_DISABLE_WERROR True)
SETUP_PROJECT()

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES talos_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Find xml_reflection
ADD_REQUIRED_DEPENDENCY("urdfdom >= 0.2.9")

FOREACH(dir config gazebo meshes robots urdf srdf)
  INSTALL(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
ENDFOREACH(dir)

SETUP_PROJECT_FINALIZE()
