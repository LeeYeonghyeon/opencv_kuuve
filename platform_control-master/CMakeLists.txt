cmake_minimum_required(VERSION 3.0.2)
project(platform_control)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  ackermann_msgs
)

#find_package(Boost REQUIRED COMPONENTS system)


# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(write_serial  src/write_serial.cpp)
#add_executable(read_serial  src/read_serial.cpp)
target_link_libraries(write_serial  ${catkin_LIBRARIES})
#target_link_libraries(read_serial ${catkin_LIBRARIES})
