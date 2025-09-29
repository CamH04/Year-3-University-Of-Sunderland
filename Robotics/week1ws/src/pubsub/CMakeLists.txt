cmake_minimum_required(VERSION 3.8)
project(pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# launch file
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#add new files here dumbo
add_executable(pubnode src/pubnode.cpp)
ament_target_dependencies(pubnode rclcpp std_msgs)
install(TARGETS
  pubnode
  DESTINATION lib/${PROJECT_NAME})

add_executable(subnode src/subnode.cpp)
ament_target_dependencies(subnode rclcpp std_msgs)
install(TARGETS
  subnode
  DESTINATION lib/${PROJECT_NAME})


ament_package()
