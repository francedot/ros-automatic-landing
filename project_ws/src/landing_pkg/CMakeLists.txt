# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(landing_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build ar_sys_listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ar_sys_listener src/ar_sys_listener.cpp)
target_link_libraries(ar_sys_listener ${catkin_LIBRARIES})
add_dependencies(ar_sys_listener landing_pkg_generate_messages_cpp)

# add_executable(listener src/listener.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener landing_pkg_generate_messages_cpp)

# %EndTag(FULLTEXT)%
