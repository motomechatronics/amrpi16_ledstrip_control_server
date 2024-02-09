cmake_minimum_required(VERSION 3.8)
project(amrpi16_ledstrip_control_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(amrpi16_ledstrip_control_server_node src/amrpi16_ledstrip_control_server.cpp src/PCA9685.cpp src/I2C.cpp)
ament_target_dependencies(amrpi16_ledstrip_control_server_node rclcpp)

install(TARGETS
   amrpi16_ledstrip_control_server_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Add header folder to find header
target_include_directories(amrpi16_ledstrip_control_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# install gli header
install(DIRECTORY include/
  DESTINATION ${PROJECT_NAME}/include
)

# Install other necessary files
install(FILES
  src/I2C.cpp
  src/PCA9685.cpp
  DESTINATION lib/${PROJECT_NAME}/src
)

ament_package()

