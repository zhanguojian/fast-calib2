cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features sample_consensus registration)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

set(FAST_CALIB_DEPS
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  cv_bridge
  image_geometry
  rosbag2_cpp
  rosbag2_storage
  tf2
)

add_executable(fast_calib_node src/main.cpp)
ament_target_dependencies(fast_calib_node ${FAST_CALIB_DEPS})
target_link_libraries(fast_calib_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

add_executable(multi_fast_calib src/multi_scene.cpp)
ament_target_dependencies(multi_fast_calib ${FAST_CALIB_DEPS})
target_link_libraries(multi_fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

add_executable(lidar_center_test test/lidar_center_test.cpp)
target_include_directories(lidar_center_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
ament_target_dependencies(lidar_center_test ${FAST_CALIB_DEPS})
target_link_libraries(lidar_center_test
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(common_lib_test test/common_lib_test.cpp)
  if(TARGET common_lib_test)
    ament_target_dependencies(common_lib_test ${FAST_CALIB_DEPS})
    target_link_libraries(common_lib_test
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      "${cpp_typesupport_target}"
    )
  endif()
endif()

install(TARGETS
  fast_calib_node
  multi_fast_calib
  lidar_center_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz_cfg
  calib_data
  output
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
