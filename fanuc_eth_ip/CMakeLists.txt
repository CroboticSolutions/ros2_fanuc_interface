cmake_minimum_required(VERSION 3.22)
project(fanuc_eth_ip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)

find_library(EIPSCANNER_LIBRARY NAMES EIPScanner PATHS /usr/local/lib REQUIRED)

add_library(fanuc_eth_ip SHARED src/fanuc_eth_ip.cpp)
target_compile_features(fanuc_eth_ip PUBLIC cxx_std_17)
target_include_directories(fanuc_eth_ip PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/local/include
  /usr/local/include/EIPScanner
)
target_link_libraries(fanuc_eth_ip PUBLIC ${EIPSCANNER_LIBRARY})
ament_target_dependencies(fanuc_eth_ip PUBLIC rcpputils rclcpp)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS fanuc_eth_ip
  EXPORT fanuc_eth_ip
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(fanuc_eth_ip HAS_LIBRARY_TARGET)
ament_export_dependencies(rcpputils rclcpp)

ament_package()
