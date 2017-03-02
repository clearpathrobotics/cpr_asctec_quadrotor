cmake_minimum_required(VERSION 2.8.3)
project(cpr_asctec_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(
  DIRECTORY launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch)
  roslaunch_add_file_check(launch)
endif()
