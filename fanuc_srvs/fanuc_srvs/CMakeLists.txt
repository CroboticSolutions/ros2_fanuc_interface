cmake_minimum_required(VERSION 3.22)
project(fanuc_srvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fanuc_eth_ip REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(fanuc_srvs_msgs REQUIRED)

add_executable(fanuc_srvs src/fanuc_srvs.cpp)
target_compile_features(fanuc_srvs PUBLIC cxx_std_17)
ament_target_dependencies(fanuc_srvs fanuc_srvs_msgs rclcpp std_srvs)
target_link_libraries(fanuc_srvs fanuc_eth_ip::fanuc_eth_ip)

install(TARGETS
  fanuc_srvs
  DESTINATION lib/${PROJECT_NAME})

ament_package()
