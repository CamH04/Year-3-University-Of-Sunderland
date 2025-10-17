cmake_minimum_required(VERSION 3.8)
project(turtlesim_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




add_executable(turtle src/turtle_controller.cpp)
ament_target_dependencies(turtle rclcpp geometry_msgs)
install(TARGETS
  turtle
  DESTINATION lib/${PROJECT_NAME})






add_executable(vturtle src/turtle_vis.cpp)
ament_target_dependencies(vturtle rclcpp geometry_msgs)
install(TARGETS
  vturtle
  DESTINATION lib/${PROJECT_NAME})
ament_package()
