cmake_minimum_required(VERSION 2.8.3)
project(wrench_transformer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy std_msgs tf)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system iostreams thread)

SET(CMAKE_BUILD_TYPE debug)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetForceTorque.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs  
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
# add_dependencies(ipa325_wrench_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(WrenchTransformer src/WrenchTransformer.cpp)
add_executable(WrenchCollector src/WrenchCollector.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(WrenchTransformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(WrenchCollector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(WrenchTransformer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(WrenchCollector
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
#############
## Install ##
#############

install(TARGETS WrenchTransformer WrenchCollector
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
