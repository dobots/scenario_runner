cmake_minimum_required(VERSION 3.5)
project(bt_demo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

#Add the executable and name it so you can run your node using ros2 run
add_executable(batterycheck src/batterycheck.cpp)
ament_target_dependencies(batterycheck rclcpp std_msgs behaviortree_cpp_v3)

add_executable(batterycheck_logger src/batterycheck_logger.cpp)
ament_target_dependencies(batterycheck_logger rclcpp std_msgs behaviortree_cpp_v3)

add_executable(t01_create_tree src/t01_create_tree.cpp)
ament_target_dependencies(t01_create_tree rclcpp std_msgs behaviortree_cpp_v3)


#Finally, add the install(TARGETS..) section so ros2 run can find your executable
install(TARGETS batterycheck DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS batterycheck_logger DESTINATION lib/${PROJECT_NAME})

install(TARGETS t01_create_tree DESTINATION lib/${PROJECT_NAME})

ament_package()
