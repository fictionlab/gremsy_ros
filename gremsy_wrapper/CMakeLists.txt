cmake_minimum_required(VERSION 3.8)
project(gremsy_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gremsy_sdk REQUIRED)

generate_parameter_library(
  gremsy_wrapper_parameters
  src/gremsy_wrapper_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/gremsy_wrapper.cpp
)

target_link_libraries(${PROJECT_NAME}
  gremsy_wrapper_parameters
  rclcpp::rclcpp
  rclcpp_components::component
)

ament_target_dependencies(${PROJECT_NAME}
  sensor_msgs
  std_srvs
  geometry_msgs
  gremsy_sdk
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "gremsy_wrapper::GremsyWrapper"
  EXECUTABLE "gremsy_wrapper_node"
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
