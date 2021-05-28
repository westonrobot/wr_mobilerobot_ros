cmake_minimum_required(VERSION 2.8.3)
project(robot_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(APPEND CMAKE_PREFIX_PATH "/opt/weston_robot/lib/cmake")

find_package(catkin REQUIRED COMPONENTS    
    roslaunch 
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    tf2 
    tf2_ros    
    robot_msgs
)

find_package(wrp_sdk REQUIRED)
find_package(zbus REQUIRED)

# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scout_messenger
  CATKIN_DEPENDS robot_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(scout_messenger STATIC src/scout_messenger.cpp)
target_link_libraries(scout_messenger ${catkin_LIBRARIES} westonrobot::wrp_sdk westonrobot::zbus)
add_dependencies(scout_messenger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scout_base_node src/scout_base_node.cpp)
target_link_libraries(scout_base_node scout_messenger ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS scout_messenger scout_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
