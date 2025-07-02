cmake_minimum_required(VERSION 3.8)
project(command_dyposition)

find_package(rosidl_default_generators REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CommandValue.srv"
)

add_executable(server src/dycontrol_server.cpp)
ament_target_dependencies(server
  dynamixel_workbench_toolbox
  rclcpp)

add_executable(client src/dycontrol_client.cpp)
ament_target_dependencies(client
  dynamixel_workbench_toolbox
  rclcpp)


rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(server "${cpp_typesupport_target}")
target_link_libraries(client "${cpp_typesupport_target}")




ament_export_dependencies(rosidl_default_runtime)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package()
