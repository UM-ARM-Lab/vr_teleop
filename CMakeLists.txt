cmake_minimum_required(VERSION 3.5)
project(openvr_driver)

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

# Include OpenVR
set(openvr_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/openvr/headers")

if (WIN32)
  set(openvr_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/openvr/lib/win64")
  set(openvr_BINARY_DIR "${CMAKE_SOURCE_DIR}/openvr/bin/win64")
elseif (UNIX)
  set(openvr_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/openvr/lib/linux64")
  set(openvr_BINARY_DIR "${CMAKE_SOURCE_DIR}/openvr/bin/linux64")
endif()

include_directories(SYSTEM ${openvr_INCLUDE_DIR})
link_directories(${openvr_LIBRARY_DIR})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vive_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Add cpp file
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "sensor_msgs"
  "vive_msgs"
  "tf2"
)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})
  
target_link_libraries(${PROJECT_NAME}_node openvr_api)

# (hack) Install OpenVR api dll
install(FILES ${openvr_BINARY_DIR}/openvr_api.dll
  DESTINATION lib/${PROJECT_NAME})

# Testing config
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
