cmake_minimum_required(VERSION 3.8)
project(exmple_dynamixel_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

include_directories(
  include
  ${dynamixel_workbench_toolbox_INCLUDE_DIRS}
)

add_executable(dynamixel_controller src/dynamixel_controller.cpp)

ament_target_dependencies(dynamixel_controller
  rclcpp
  dynamixel_workbench_toolbox
)

install(TARGETS
  dynamixel_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
