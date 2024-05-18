cmake_minimum_required(VERSION 3.0.2)
project(effector_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2_ros
  orocos_kdl
  kdl_parser
  sensor_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES effector_control
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  src/slider_gui.py
  src/slider_service.py
  scripts/follow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(gazebo REQUIRED)
# find_package(Qt4Widgets)
include_directories(${GAZEBO_INCLUDE_DIRS} include /usr/include/qt4)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(force_application SHARED src/force_application.cc)
target_link_libraries(force_application ${GAZEBO_LIBRARIES})

add_executable(ee_transform_node src/end_effector_force.cc)
target_link_libraries(ee_transform_node ${catkin_LIBRARIES})
add_dependencies(ee_transform_node roscpp orocos_kdl kdl_parser sensor_msgs geometry_msgs)
