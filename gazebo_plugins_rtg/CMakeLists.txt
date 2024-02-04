cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugins_rtg)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  std_msgs
  nav_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
  LIBRARIES ${PROJECT_NAME}
)


add_library(hello_world_plugin src/simple_world_plugin.cpp)
target_link_libraries(hello_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(hello_model_plugin src/simple_model_plugin.cpp)
target_link_libraries(hello_model_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(hello_arm_plugin src/simple_arm_plugin.cpp)
target_link_libraries(hello_arm_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(hello_ur10_short_plugin src/simple_ur10_short_plugin.cpp)
target_link_libraries(hello_ur10_short_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(ur10_plugin src/simple_ur10.cpp)
target_link_libraries(ur10_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(cmd src/cmd.cpp)
target_link_libraries(cmd ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
