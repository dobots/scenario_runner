cmake_minimum_required(VERSION 3.5)
project(scenarios)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  gazebo_ros_pkgs
  gazebo_msgs
  behaviortree_cpp_v3
  ${OTHER_DEPS}
)  

#Add the executable and name it so you can run your node using ros2 run
add_executable(ch_class1 src/ch_class1.cpp)
ament_target_dependencies(ch_class1 rclcpp std_msgs behaviortree_cpp_v3)

add_executable(ch_class2 src/ch_class2.cpp)
ament_target_dependencies(ch_class2 rclcpp std_msgs behaviortree_cpp_v3)

add_executable(ch_class3 src/ch_class3.cpp)
ament_target_dependencies(ch_class3 rclcpp std_msgs gazebo_ros_pkgs gazebo_msgs behaviortree_cpp_v3)

add_executable(spawn_random_pos src/spawn_random_pos.cpp)
ament_target_dependencies(spawn_random_pos rclcpp std_msgs gazebo_ros_pkgs gazebo_msgs behaviortree_cpp_v3)

add_executable(spawn_multiple src/spawn_multiple.cpp)
ament_target_dependencies(spawn_multiple rclcpp std_msgs gazebo_ros_pkgs gazebo_msgs behaviortree_cpp_v3)

add_executable(spawn_rand_max src/spawn_rand_max.cpp)
ament_target_dependencies(spawn_rand_max rclcpp std_msgs gazebo_ros_pkgs gazebo_msgs behaviortree_cpp_v3)

add_executable(spawn_prox src/spawn_prox.cpp)
ament_target_dependencies(spawn_prox rclcpp std_msgs gazebo_ros_pkgs gazebo_msgs behaviortree_cpp_v3)

add_executable(ch_class4 src/ch_class4.cpp)
ament_target_dependencies(ch_class4 rclcpp std_msgs behaviortree_cpp_v3)




install(
  DIRECTORY include launch scenarios src worlds xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  launch/load_empty_world_gazebo.launch.py
  DESTINATION lib/${PROJECT_NAME}
)


#Finally, add the install(TARGETS..) section so ros2 run can find your executable
install(TARGETS ch_class1 DESTINATION lib/${PROJECT_NAME})
install(TARGETS ch_class2 DESTINATION lib/${PROJECT_NAME})
install(TARGETS ch_class3 DESTINATION lib/${PROJECT_NAME})
install(TARGETS spawn_random_pos DESTINATION lib/${PROJECT_NAME})
install(TARGETS spawn_multiple DESTINATION lib/${PROJECT_NAME})
install(TARGETS spawn_rand_max DESTINATION lib/${PROJECT_NAME})
install(TARGETS spawn_prox DESTINATION lib/${PROJECT_NAME})
install(TARGETS ch_class4 DESTINATION lib/${PROJECT_NAME})

ament_package()
