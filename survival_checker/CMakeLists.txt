cmake_minimum_required(VERSION 2.8.3)
project(survival_checker)

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  autoware_msgs
  topic_tools
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  roslib
  std_msgs
  autoware_msgs
  topic_tools
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
  src/survival_checker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
add_dependencies(test
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY params/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)
