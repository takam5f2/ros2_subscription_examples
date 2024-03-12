cmake_minimum_required(VERSION 3.5)

project(intra_process_examples)

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
ament_export_include_directories("include/${PROJECT_NAME}")


add_executable(two_node_pipeline_timer
  src/two_node_pipeline_timer.cpp)
target_link_libraries(two_node_pipeline_timer
  rclcpp::rclcpp
  ${std_msgs_TARGETS})


install(TARGETS
  two_node_pipeline_timer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)
  # Add each test case.  Multi-executable tests can be specified in
  # semicolon-separated strings, like  exe1;exe2.
endif()


ament_package()
