cmake_minimum_required(VERSION 3.8)
project(raspberry_gpio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(ras_gpio_node src/ras_gpio_node.cpp)
ament_target_dependencies(ras_gpio_node rclcpp std_msgs)
target_link_libraries(ras_gpio_node wiringPi) 

add_executable(multi_gpio_control_node src/multi_gpio_control_node.cpp)
ament_target_dependencies(multi_gpio_control_node rclcpp std_msgs)
target_link_libraries(multi_gpio_control_node wiringPi) 


include_directories(include)
include_directories(/usr/include)

# Install targets
install(TARGETS 
  ras_gpio_node
  multi_gpio_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
  )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()