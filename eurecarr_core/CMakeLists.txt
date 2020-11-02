cmake_minimum_required(VERSION 2.8.3)
project(eurecarr_core)


find_package(catkin REQUIRED COMPONENTS
  autorally_msgs
  roscpp
  rospy
  rosserial_arduino
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED)

generate_dynamic_reconfigure_options(
   config/MessageInterfaceParams.cfg
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
	DEPENDS
	CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs image_transport autorally_msgs
	INCLUDE_DIRS include
	LIBRARIES SerialSensorInterface ar_diagnostics RingBuffer	
)

###########
## Build ##
###########

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(include)

add_subdirectory(src/MessageInterface)
#############
## Install ##
#############
install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml"
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_eurecarr_core.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
