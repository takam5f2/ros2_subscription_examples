cmake_minimum_required(VERSION 3.5)

project(simple_examples)

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
    PRIVATE "simple_examples_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rclcpp_components"
    "rcutils"
    "std_msgs")
endfunction()

# Tutorials of Publish/Subscribe with Topics
add_library(simple_examples_library SHARED
  src/talker.cpp
  src/timer_listener.cpp
  src/timer_listener_twice.cpp
  src/timer_listener_using_callback.cpp
  src/timer_listener_using_normal_function.cpp
  src/timer_batch_listener.cpp
  src/talker_serialized_message.cpp
  src/timer_listener_serialized_message.cpp)
add_demo_dependencies(simple_examples_library)



rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::Talker"
  EXECUTABLE talker)
rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::TimerListener"
  EXECUTABLE timer_listener)
rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::TimerListenerTwice"
  EXECUTABLE timer_listener_twice)
rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::TimerListenerUsingCallback"
  EXECUTABLE timer_listener_using_callback)
rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::TimerListenerUsingNormalFunction"
  EXECUTABLE timer_listener_using_normal_function)
rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::TimerBatchListener"
  EXECUTABLE timer_batch_listener)
rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::SerializedMessageTalker"
  EXECUTABLE serialized_message_talker)
rclcpp_components_register_node(simple_examples_library
  PLUGIN "simple_examples::TimerSerializedMessageListener"
  EXECUTABLE timer_serialized_message_listener)


install(TARGETS
  simple_examples_library
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
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
