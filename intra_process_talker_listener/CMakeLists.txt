cmake_minimum_required(VERSION 3.5)

project(intra_process_talker_listener)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

function(add_demo_dependencies library)
  target_compile_definitions(${library}
    PRIVATE "INTRA_PROCESS_TALKER_LISTENER_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rclcpp_components"
    "rcutils"
    "std_msgs")
endfunction()

# Tutorials of Publish/Subscribe with Topics
add_library(INTRA_PROCESS_TALKER_LISTENER_library SHARED
  src/talker_intra_process.cpp
  src/timer_listener_intra_process.cpp
  src/timer_batch_listener_intra_process.cpp)
add_demo_dependencies(INTRA_PROCESS_TALKER_LISTENER_library)


rclcpp_components_register_node(INTRA_PROCESS_TALKER_LISTENER_library
  PLUGIN "intra_process_talker_listener::TalkerIntraProcess"
  EXECUTABLE talker_intra_process)
rclcpp_components_register_node(INTRA_PROCESS_TALKER_LISTENER_library
  PLUGIN "intra_process_talker_listener::TimerListenerIntraProcess"
  EXECUTABLE timer_listener_intra_process)
rclcpp_components_register_node(INTRA_PROCESS_TALKER_LISTENER_library
  PLUGIN "intra_process_talker_listener::TimerBatchListenerIntraProcess"
  EXECUTABLE timer_batch_listener_intra_process)

install(TARGETS
  INTRA_PROCESS_TALKER_LISTENER_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)
  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1;exe2.
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
