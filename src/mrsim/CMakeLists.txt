cmake_minimum_required(VERSION 3.0.2)
project(mrsim)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
 INCLUDE_DIRS include
 # LIBRARIES
 # CATKIN_DEPENDS
 # DEPENDS
)

##########
# Build ##
##########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(mrsim_node src/main.cpp src/lidar.cpp src/simple_geometry.cpp src/world.cpp src/robot.cpp)

add_dependencies(mrsim_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(mrsim_node ${catkin_LIBRARIES} ${OpenCV_LIBS} jsoncpp)
