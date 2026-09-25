cmake_minimum_required(VERSION 3.22)
project(fanuc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fanuc_eth_ip REQUIRED)
find_package(fanuc_rmi REQUIRED)

include_directories(/usr/local/include/EIPScanner)

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

add_library(fanuc SHARED src/fanuc_hw.cpp)
target_compile_features(fanuc PUBLIC cxx_std_17)
target_include_directories(fanuc PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/fanuc>
)

ament_target_dependencies(fanuc PUBLIC ${HW_IF_INCLUDE_DEPENDS} sensor_msgs)
target_link_libraries(fanuc PUBLIC fanuc_eth_ip::fanuc_eth_ip fanuc_rmi::rmi_driver)

pluginlib_export_plugin_description_file(hardware_interface hw_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/fanuc
)

install(TARGETS fanuc
  EXPORT fanuc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(dpm src/dpm.cpp)
target_compile_features(dpm PUBLIC cxx_std_17)
ament_target_dependencies(
  dpm
  ${HW_IF_INCLUDE_DEPENDS}
  std_srvs
  std_msgs
  sensor_msgs
)
target_link_libraries(dpm fanuc_eth_ip::fanuc_eth_ip)
install(TARGETS dpm
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(fanuc HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} fanuc_eth_ip fanuc_rmi)

ament_package()
