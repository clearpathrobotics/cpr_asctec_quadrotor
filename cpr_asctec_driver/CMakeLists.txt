cmake_minimum_required(VERSION 2.8.3)
project(cpr_asctec_driver)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  hector_quadrotor_interface
  hector_uav_msgs
  roscpp
  serial
  )
find_package(Boost REQUIRED COMPONENTS chrono)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;")

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cpr_asctec_hardware
  CATKIN_DEPENDS hector_uav_msgs roscpp serial
  )

add_library(cpr_asctec_hardware
  src/asctec_hardware.cpp
  src/asctec_serial.cpp
  )
target_link_libraries(cpr_asctec_hardware ${catkin_LIBRARIES})

add_executable(cpr_asctec_node
  src/asctec_node.cpp
  )
target_link_libraries(cpr_asctec_node cpr_asctec_hardware ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS cpr_asctec_node cpr_asctec_hardware
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslint_cpp()
  roslint_add_test()
  roslaunch_add_file_check(launch)
endif()
