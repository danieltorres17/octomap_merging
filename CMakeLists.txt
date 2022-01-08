cmake_minimum_required(VERSION 3.0.2)
project(octomap_merging)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  message_generation
  nav_msgs
  octomap_ros
  octomap_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  rough_octomap
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_message_files(
  FILES
  OctomapArray.msg
  OctomapNeighbors.msg
)

generate_messages(
  DEPENDENCIES
  octomap_msgs
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS marble_mapping message_runtime
  CATKIN_DEPENDS message_runtime
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(octomap_merging src/octomap_merging.cpp)
target_link_libraries(octomap_merging ${catkin_LIBRARIES} ${LINK_LIBS})
add_dependencies(octomap_merging ${catkin_EXPORTED_TARGETS})

add_executable(octomap_merging_node src/octomap_merging_node.cpp)
target_link_libraries(octomap_merging_node octomap_merging ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)