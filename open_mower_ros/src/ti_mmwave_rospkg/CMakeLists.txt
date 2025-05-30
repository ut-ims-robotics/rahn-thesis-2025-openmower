cmake_minimum_required(VERSION 3.10)
project(ti_mmwave_rospkg)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

# Find catkin macros and libraries
find_package(catkin REQUIRED 
  COMPONENTS
    nodelet
    roscpp
    serial
    std_msgs
    sensor_msgs
    message_generation
    pluginlib
    pcl_ros
    roslint
)

## Find system libraries
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
  FILES
    RadarScan.msg
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
  FILES
    mmWaveCLI.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    mmwave
  CATKIN_DEPENDS
    nodelet
    roscpp
    serial
    std_msgs
  DEPENDS
    # system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ti_mmwave_rospkg_INCLUDE_DIRS}
  # ${pthread_INCLUDE_DIRS}
  # ${rt_INCLUDE_DIRS} # (?)
)

## mmwave library
# 1. declare cpp library
add_library(mmwave
  src/mmWaveDataHdl.cpp
  src/mmWaveCommSrv.cpp
  src/DataHandlerClass.cpp
  src/mmWaveQuickConfig.cpp
  src/ParameterParser.cpp
)
# 2. add dependencies
add_dependencies(mmwave
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
# 3. specify libraries to link executable targets against
target_link_libraries(mmwave
  ${serial_LIBRARIES}
  Threads::Threads
)

## executable 1: ti_mmwave_rospkg (==${PROJECT_NAME}) executable
# 1. Declare cpp executables
add_executable(${PROJECT_NAME} 
  src/mmWaveLoader.cpp
  include/ti_mmwave_rospkg/point_types.h
)
# 2. add dependencies
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  ${serial_EXPORTED_TARGETS}
  mmwave
)
# 3. specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${serial_LIBRARIES}
  mmwave
  Threads::Threads
)

## same for executable 2: mmWaveQuickConfig
add_executable(mmWaveQuickConfig
  src/mmWaveQuickConfig.cpp
)
target_link_libraries(
  mmWaveQuickConfig 
  ${catkin_LIBRARIES}
  mmwave
)
add_dependencies(mmWaveQuickConfig
  ${catkin_EXPORTED_TARGETS}
  mmwave
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ti_mmwave_rospkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()



##########################
## Static code analysis ##
##########################

## usage: catkin build --this --no-deps --make-args roslint
roslint_cpp()
