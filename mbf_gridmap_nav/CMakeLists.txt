cmake_minimum_required(VERSION 2.8.3)
project(mbf_gridmap_nav)

if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set (CMAKE_CXX_FLAGS "-std=gnu++11 ${CMAKE_CXX_FLAGS}")
    endif ()
else ()
    set (CMAKE_CXX_STANDARD 11)
endif ()

find_package(catkin REQUIRED
  COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  grid_map_core
  grid_map_handler
  mbf_abstract_nav
  mbf_gridmap_core
  mbf_msgs
  mbf_utility
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

find_package(Boost COMPONENTS thread chrono REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/MoveBaseFlex.cfg
)

set(MBF_GRIDMAP_SERVER_LIB mbf_gridmap_server)
set(MBF_GRIDMAP_SERVER_NODE mbf_gridmap_nav)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${MBF_GRIDMAP_SERVER_LIB}
  CATKIN_DEPENDS
  dynamic_reconfigure
  grid_map_core
  grid_map_handler
  geometry_msgs
  mbf_abstract_nav
  mbf_gridmap_core
  mbf_msgs
  mbf_utility
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${MBF_GRIDMAP_SERVER_LIB}
  src/mbf_gridmap_nav/gridmap_navigation_server.cpp
  src/mbf_gridmap_nav/gridmap_planner_execution.cpp
  src/mbf_gridmap_nav/gridmap_controller_execution.cpp
  src/mbf_gridmap_nav/gridmap_recovery_execution.cpp
)
add_dependencies(${MBF_GRIDMAP_SERVER_LIB} ${catkin_EXPORTED_TARGETS})
add_dependencies(${MBF_GRIDMAP_SERVER_LIB} ${PROJECT_NAME}_gencfg)

target_link_libraries(${MBF_GRIDMAP_SERVER_LIB}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${MBF_GRIDMAP_SERVER_NODE} src/gridmap_server_node.cpp)
add_dependencies(${MBF_GRIDMAP_SERVER_NODE} ${MBF_GRIDMAP_SERVER_LIB})
target_link_libraries(${MBF_GRIDMAP_SERVER_NODE}
  ${MBF_GRIDMAP_SERVER_LIB}
  ${catkin_LIBRARIES}
)

install(TARGETS
  ${MBF_GRIDMAP_SERVER_LIB} ${MBF_GRIDMAP_NODE}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
