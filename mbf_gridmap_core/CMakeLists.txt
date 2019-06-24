cmake_minimum_required(VERSION 2.8.3)
project(mbf_gridmap_core)

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  mbf_abstract_core
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES mbf_gridmap_core
   CATKIN_DEPENDS grid_map_core mbf_abstract_core
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)