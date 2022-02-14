cmake_minimum_required(VERSION 3.8)
project(pubsub_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(crane_commander src/crane_commander.cpp)
ament_target_dependencies(crane_commander rclcpp std_msgs)

# Install targets
install(TARGETS
  crane_commander
  DESTINATION lib/${PROJECT_NAME})

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)


ament_package()
