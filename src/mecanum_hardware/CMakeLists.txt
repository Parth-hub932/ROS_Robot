cmake_minimum_required(VERSION 3.8)
project(mecanum_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# 2. Find LibSerial (installed from apt)
find_path(LIBSERIAL_INCLUDE_DIR SerialStream.h
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES serial Serial libserial
  REQUIRED)

find_library(LIBSERIAL_LIBRARY
  NAMES serial libserial
  PATHS /usr/lib /usr/lib/aarch64-linux-gnu /usr/local/lib
  REQUIRED)

# Debug message (optional)
message(STATUS "LibSerial include dir: ${LIBSERIAL_INCLUDE_DIR}")
message(STATUS "LibSerial library: ${LIBSERIAL_LIBRARY}")

# 3. Create the Plugin Library
add_library(${PROJECT_NAME} SHARED
  src/mecanum_system.cpp
)

# 4. Include Directories
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  ${LIBSERIAL_INCLUDE_DIR}
)

# 5. Link Libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${LIBSERIAL_LIBRARY}
)

# 6. Export Plugin
pluginlib_export_plugin_description_file(hardware_interface mecanum_hardware.xml)

# 7. Install Rules
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES mecanum_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
