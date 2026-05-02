cmake_minimum_required(VERSION 3.8)
project(lego_finder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ONNX Runtime - deve stare PRIMA di usare la variabile
set(ONNXRUNTIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)

add_executable(lego_finder src/lego_finder.cpp)

ament_target_dependencies(lego_finder 
  rclcpp geometry_msgs sensor_msgs interfaces message_filters cv_bridge tf2)

target_include_directories(lego_finder PUBLIC 
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_DIR}/include)

target_link_libraries(lego_finder 
  ${OpenCV_LIBRARIES} 
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)

install(TARGETS
  lego_finder
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
