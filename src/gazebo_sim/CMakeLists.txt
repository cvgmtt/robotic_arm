cmake_minimum_required(VERSION 3.8)
project(gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trovo il sistema di build
find_package(ament_cmake REQUIRED)


install(DIRECTORY
  worlds
  launch
  lego_models
  config
  DESTINATION share/${PROJECT_NAME}
)

# Chiudo il pacchetto
ament_package()