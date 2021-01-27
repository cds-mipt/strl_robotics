cmake_minimum_required(VERSION 2.8.3)
project(alego)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nodelet
  tf
  pcl_conversions
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

add_executable(IP src/IP.cpp)
target_link_libraries(IP ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})

add_dependencies(IP ${PROJECT_NAME}_generate_messages_cpp)

add_executable(LO src/LO.cpp)
target_link_libraries(LO ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN3_LIBRARIES})

add_dependencies(LO ${PROJECT_NAME}_generate_messages_cpp)

add_executable(LM src/LM.cpp)
target_link_libraries(LM ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN3_LIBRARIES} gtsam)

add_dependencies(LM ${PROJECT_NAME}_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See https://docs.ros.org/melodic/api/catkin/html/user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS IP
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS LO
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS LM
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

## Mark other files for installation (e.g. launch and configuration files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)