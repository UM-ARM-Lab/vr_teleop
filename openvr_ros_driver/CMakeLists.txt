cmake_minimum_required(VERSION 2.8.3)
project(openvr_ros_driver)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

# Include OpenVR
set(openvr_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/openvr/headers")
set(openvr_LIBRARY_DIR "${PROJECT_SOURCE_DIR}/openvr/lib/linux64")

include_directories(SYSTEM ${openvr_INCLUDE_DIR})
link_directories(${openvr_LIBRARY_DIR})

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  vive_msgs
)

include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/openvr_driver.cpp src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  openvr_api
)

