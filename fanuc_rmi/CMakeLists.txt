cmake_minimum_required(VERSION 3.22)
project(fanuc_rmi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcpputils REQUIRED)

add_library(rmi_communication SHARED src/rmi_communication.cpp)
target_compile_features(rmi_communication PUBLIC cxx_std_17)
target_include_directories(rmi_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rmi_communication PUBLIC rclcpp std_msgs rcpputils)

add_library(rmi_driver SHARED src/rmi_driver.cpp)
target_compile_features(rmi_driver PUBLIC cxx_std_17)
target_include_directories(rmi_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rmi_driver PUBLIC rmi_communication)
ament_target_dependencies(rmi_driver PUBLIC rclcpp std_msgs rcpputils)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS rmi_communication rmi_driver
  EXPORT fanuc_rmi
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

add_executable(rmi src/rmi.cpp)
target_compile_features(rmi PUBLIC cxx_std_17)
target_link_libraries(rmi rmi_driver)
ament_target_dependencies(rmi rclcpp std_msgs rcpputils)
install(TARGETS rmi DESTINATION lib/${PROJECT_NAME})

ament_export_targets(fanuc_rmi HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs rcpputils)

ament_package()
