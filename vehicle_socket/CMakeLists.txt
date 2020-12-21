cmake_minimum_required(VERSION 2.8.3)
project(vehicle_socket)

find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_can_msgs
  autoware_config_msgs
  autoware_system_msgs
  autoware_msgs
  roscpp
  std_msgs
  tablet_socket_msgs
  geometry_msgs
  nmea_msgs
  roslib
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_receiver nodes/vehicle_receiver/vehicle_receiver.cpp)
target_link_libraries(vehicle_receiver ${catkin_LIBRARIES})
add_dependencies(vehicle_receiver ${catkin_EXPORTED_TARGETS})

add_executable(vehicle_sender nodes/vehicle_sender/vehicle_sender.cpp)
target_link_libraries(vehicle_sender ${catkin_LIBRARIES})
add_dependencies(vehicle_sender ${catkin_EXPORTED_TARGETS})

add_executable(kvaser_microbus_can_receiver
	nodes/kvaser_microbus_can_receiver/kvaser_microbus_can_receiver.cpp)
target_link_libraries(kvaser_microbus_can_receiver
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_receiver
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_microbus_can_sender
	nodes/kvaser_microbus_can_sender/kvaser_microbus_can_sender_stroke.cpp)
target_link_libraries(kvaser_microbus_can_sender
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_sender
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_microbus_can_receiver111scw
	nodes/kvaser_microbus_can_receiver111scw/kvaser_microbus_can_receiver.cpp)
target_link_libraries(kvaser_microbus_can_receiver111scw
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_receiver111scw
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_microbus_can_sender111scw
	nodes/kvaser_microbus_can_sender111scw/kvaser_microbus_can_sender_stroke.cpp)
target_link_libraries(kvaser_microbus_can_sender111scw
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_sender111scw
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(microbus_pseudo_can_publisher
	nodes/microbus_pseudo_can_publisher/microbus_pseudo_can_publisher.cpp)
target_link_libraries(microbus_pseudo_can_publisher
  ${catkin_LIBRARIES}
  )
add_dependencies(microbus_pseudo_can_publisher
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(microbus_pseudo_can_sender
	nodes/microbus_pseudo_can_sender/microbus_pseudo_can_sender.cpp)
target_link_libraries(microbus_pseudo_can_sender
  ${catkin_LIBRARIES}
  )
add_dependencies(microbus_pseudo_can_sender
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS vehicle_sender vehicle_receiver kvaser_microbus_can_sender kvaser_microbus_can_receiver
                kvaser_microbus_can_sender111scw kvaser_microbus_can_receiver111scw
                microbus_pseudo_can_publisher microbus_pseudo_can_sender
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
