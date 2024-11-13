cmake_minimum_required(VERSION 3.8)
project(gremsy_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(mavlink REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/gimbal_interface.cpp
  src/gimbal_protocol_v1.cpp
  src/gimbal_protocol_v2.cpp
  src/gimbal_protocol.cpp
  src/serial_port.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  mavlink
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(mavlink_control
  src/examples/mavlink_control.cpp
)
target_link_libraries(mavlink_control
  ${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  TARGETS
    mavlink_control
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
