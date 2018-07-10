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

# file(COPY ${openvr_BINARY_DIR}/openvr_api.dll DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rcutils"
    "sensor_msgs")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(topics talkerJoy)
target_link_libraries(talkerJoy openvr_api)
custom_executable(topics listenerJoy)

install(FILES ${openvr_BINARY_DIR}/openvr_api.dll
  DESTINATION lib/${PROJECT_NAME})

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
