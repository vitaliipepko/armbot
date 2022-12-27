cmake_minimum_required(VERSION 3.0.2)
project(armbot_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  hardware_interface
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_service_files(
  FILES
  AnglesConverter.srv
)

# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(angles_converter_cpp src/angles_converter.cpp)
add_dependencies(angles_converter_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angles_converter_cpp ${catkin_LIBRARIES})

add_executable(armbot_interface_node src/armbot_interface.cpp)
add_dependencies(armbot_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(armbot_interface_node ${catkin_LIBRARIES})