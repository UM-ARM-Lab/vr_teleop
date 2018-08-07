cmake_minimum_required(VERSION 2.8.3)
project(openvr_ros_driver)

# Default to C99
#if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 99)
#endif()

# Default to C++14
#if(NOT CMAKE_CXX_STANDARD)
#  set(CMAKE_CXX_STANDARD 14)
#endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

# Include OpenVR
set(openvr_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/openvr/headers")
set(openvr_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/openvr/lib/linux64")
#set(openvr_BINARY_DIR "${PROJECT_SOURCE_DIR}/openvr/bin/linux64")

include_directories(SYSTEM ${openvr_INCLUDE_DIR})
link_directories(${openvr_LIBRARY_DIR})

# Find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(rmw REQUIRED)
#find_package(sensor_msgs REQUIRED)
#find_package(vive_msgs REQUIRED)
#find_package(tf2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  vive_msgs
)

# Add cpp file
#add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
#ament_target_dependencies(${PROJECT_NAME}_node
#  "rclcpp"
#  "sensor_msgs"
#  "vive_msgs"
#  "tf2"
#)

include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
)
# (hack) Install OpenVR api dll
#install(FILES ${openvr_BINARY_DIR}/openvr_api.dll
#        DESTINATION lib/${PROJECT_NAME})

#install(TARGETS src/${PROJECT_NAME}_node
#  DESTINATION lib/${PROJECT_NAME})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  openvr_api
)

