cmake_minimum_required(VERSION 2.8)
project(rcprg_gazebo_utils)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

catkin_install_python(PROGRAMS
  scripts/gazebo_move_object.py
  scripts/gazebo_publish_ros_tf_object.py
  scripts/create_state_snapshot
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

