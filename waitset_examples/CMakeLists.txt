cmake_minimum_required(VERSION 3.5)

project(waitset_examples)

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
    PRIVATE "waitset_examples_BUILDING_DLL")
  ament_target_dependencies(${library}
    "example_interfaces"
    "rclcpp"
    "rclcpp_components"
    "rcutils"
    "std_msgs")
endfunction()

# Tutorials of Publish/Subscribe with Topics
add_library(waitset_examples_library SHARED
  src/talker_triple.cpp
  src/timer_listener_single.cpp
  src/timer_listener_single_using_callback.cpp
  src/timer_batch_listener_single.cpp
  src/timer_listener_twin_static.cpp
  src/timer_listener_triple_sync.cpp
  src/timer_listener_triple_async.cpp
  src/timer_listener_triple_separated_waitset.cpp
  src/timer_listener_twin_nested_waitset.cpp
  src/timer_listener_triple_sync_intra.cpp
  src/timer_listener_single_wait_some_period.cpp)
add_demo_dependencies(waitset_examples_library)



rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TalkerTriple"
  EXECUTABLE talker_triple)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerSingle"
  EXECUTABLE timer_listener_single)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerSingleUsingCallback"
  EXECUTABLE timer_listener_single_using_callback)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerBatchListenerSingle"
  EXECUTABLE timer_batch_listener_single)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerTwinStatic"
  EXECUTABLE timer_listener_twin_static)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerTripleSync"
  EXECUTABLE timer_listener_triple_sync)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerTripleAsync"
  EXECUTABLE timer_listener_triple_async)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerTripleSeparatedWaitSet"
  EXECUTABLE timer_listener_triple_separated_waitset)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerTwinNestedWaitSet"
  EXECUTABLE timer_listener_twin_nested_waitset)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerTripleSyncIntra"
  EXECUTABLE timer_listener_triple_sync_intra)
rclcpp_components_register_node(waitset_examples_library
  PLUGIN "waitset_examples::TimerListenerSingleWaitSomePeriod"
  EXECUTABLE timer_listener_single_wait_some_period)

install(TARGETS
  waitset_examples_library
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
