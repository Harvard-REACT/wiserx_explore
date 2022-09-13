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
)
find_package(OpenMP REQUIRED)

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
  /home/react-ws-1/WSR_Project/WSR-Toolbox-cpp/include
)

link_directories(/home/react-ws-1/WSR_Project/WSR-Toolbox-cpp/wsr_build)


add_executable(explore
  src/costmap_client.cpp
  src/explore.cpp
  src/frontier_search.cpp
)
add_dependencies(explore ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(explore ${catkin_LIBRARIES})

add_executable(test_particle_filter
  src/costmap_client.cpp
  src/test_particle_filter.cpp
  src/frontier_search.cpp
)
add_dependencies(test_particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_particle_filter ${catkin_LIBRARIES} 
                      lib_wsr_1.0.1_parallel.so 
                      OpenMP::OpenMP_CXX)


# add_executable(test_wsr
#   src/test_wsr_library.cpp
# )
# add_dependencies(test_wsr ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(test_wsr ${catkin_LIBRARIES} 
#                       lib_wsr_1.0.1_parallel.so 
#                       OpenMP::OpenMP_CXX)


#############
## Install ##
#############

# # install nodes
# install(TARGETS explore
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # install roslaunch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )

# #############
# ## Testing ##
# #############
# if(CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)

#   # test all launch files
#   roslaunch_add_file_check(launch)
# endif()
