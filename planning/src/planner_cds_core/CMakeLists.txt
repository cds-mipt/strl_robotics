cmake_minimum_required(VERSION 3.0.2)
project(planner_cds_core)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cds_msgs
  nav_msgs
  roscpp
  std_msgs
)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
        #  DEPENDS other non-ROS libs
)
include_directories(
        include
        include/planner_cds_core
        ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
        src/asearch.cpp
        src/astar.cpp
        src/bfs.cpp
        src/config.cpp
        src/dijkstra.cpp
        src/environmentoptions.cpp
        src/isearch.cpp
        src/jp_search.cpp
        src/map.cpp
        src/mission.cpp
        src/theta.cpp
        src/tinyxml2.cpp
        src/xmllogger.cpp
        )


target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
        DIRECTORY include/thetastar_rrt_lib/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)