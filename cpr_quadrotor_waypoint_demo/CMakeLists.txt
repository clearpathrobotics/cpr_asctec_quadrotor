cmake_minimum_required(VERSION 2.8.3)
project(cpr_quadrotor_waypoint_demo)

find_package(catkin REQUIRED COMPONENTS
  actionlib hector_uav_msgs roscpp geometry_msgs tf2_geometry_msgs
)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    actionlib
    hector_uav_msgs
    roscpp
    geometry_msgs
    tf2_geometry_msgs
)

add_executable(path_follower
  src/path_follower.cpp
)
target_link_libraries(path_follower ${catkin_LIBRARIES})

install(TARGETS path_follower
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslint_cpp()
  roslint_add_test()
  roslaunch_add_file_check(launch)
endif()
