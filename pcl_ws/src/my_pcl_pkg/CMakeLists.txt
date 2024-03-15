cmake_minimum_required(VERSION 3.8)
project(my_pcl_pkg)

#### FROM LIMO PROJECT
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
####

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies DEP#1
find_package(ament_cmake REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(pcl_conversions REQUIRED)

include_directories(
  include
)

# DEP#2
set(DEPENDENCIES
#   "geometry_msgs"
  "nav_msgs"
  "pcl_conversions"
#   "rclcpp"
#   "std_msgs"
#   "tf2"
#   "tf2_ros"
#   "limo_msgs"
#   "sensor_msgs"
)

#### FROM LIMO PROJECT
# add_executable(limo_base 
#     src/limo_base_node.cpp
#     src/limo_driver.cpp
#     src/serial_port.cpp)
# ament_target_dependencies(limo_base rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs limo_msgs)
####

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node nav_msgs pcl_conversions) # DEP#3
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

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


#### FROM LIMO PROJECT
# install(
#   DIRECTORY include/
#   DESTINATION include
# )
# 
# install(TARGETS  
#   limo_base  
#   DESTINATION lib/${PROJECT_NAME})
# 
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/)
# 
# ament_export_include_directories(
#   include
# )

ament_package()