cmake_minimum_required(VERSION 2.8)
project(rcprg_gazebo_utils)

find_package(catkin REQUIRED COMPONENTS)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  )

catkin_package( LIBRARIES 
  breakable_joint )

add_library(breakable_joint src/breakable_joint.cpp)
target_link_libraries(breakable_joint ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS
  breakable_joint
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

catkin_install_python(PROGRAMS
  scripts/gazebo_move_object
  scripts/gazebo_publish_ros_tf_object
  scripts/create_state_snapshot
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

