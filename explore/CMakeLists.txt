cmake_minimum_required(VERSION 3.1)
project(explore_lite)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
  natnet_pkg
  wsr_exploration
  message_generation
)
find_package(OpenMP REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RangeBearing.msg
  LocalMeasurement.msg
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
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    roscpp
    std_msgs
    tf
    visualization_msgs
    wsr_exploration
    message_runtime
)

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)



## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  $ENV{HOME}/WSR_Project/WSR-Toolbox-cpp/include
)

link_directories($ENV{HOME}/WSR_Project/WSR-Toolbox-cpp/wsr_build)


add_executable(explore
  src/costmap_client.cpp
  src/explore.cpp
  src/frontier_search.cpp
  src/state_filter.cpp
)

add_executable(realtime_range_aoa
  src/realtime_range_aoa.cpp
)

add_executable(groundtruth_state_publisher
  src/groundtruth_state_publisher.cpp
)

add_dependencies(explore ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(explore ${catkin_LIBRARIES})

add_dependencies(realtime_range_aoa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(realtime_range_aoa ${catkin_LIBRARIES})


add_dependencies(groundtruth_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(groundtruth_state_publisher ${catkin_LIBRARIES})

add_executable(get_antenna_orientation src/get_antenna_orientation.cpp)
target_link_libraries(get_antenna_orientation
  ${catkin_LIBRARIES}
)

# add_executable(test_particle_filter
#   src/costmap_client.cpp
#   src/test_particle_filter.cpp
#   src/frontier_search.cpp
# )
# add_dependencies(test_particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#                 ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_particle_filter ${catkin_LIBRARIES} 
#                       lib_wsr_1.0.1_parallel.so 
#                       OpenMP::OpenMP_CXX)


# add_executable(explore_uwb_wsr
#   src/costmap_client.cpp
#   src/explore_uwb_wsr.cpp
#   src/frontier_search.cpp
# )
# add_dependencies(explore_uwb_wsr ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#                 ${catkin_EXPORTED_TARGETS})
# target_link_libraries(explore_uwb_wsr ${catkin_LIBRARIES} 
#                       lib_wsr_1.0.1_parallel.so 
#                       OpenMP::OpenMP_CXX)


# add_executable(test_wsr
#   src/test_wsr_library.cpp
# )
# add_dependencies(test_wsr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_wsr ${catkin_LIBRARIES} 
#                       lib_wsr_1.0.1_parallel.so 
#                       OpenMP::OpenMP_CXX)