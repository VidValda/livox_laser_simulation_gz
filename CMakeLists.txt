cmake_minimum_required(VERSION 3.10)
project(livox_laser_simulation_gz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-rendering8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

add_library(LivoxLidarSystem SHARED
  src/livox_lidar_system.cpp
)

target_include_directories(LivoxLidarSystem PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(LivoxLidarSystem
  gz-sim8::gz-sim8
  gz-rendering8::gz-rendering8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-sensors8::gz-sensors8
  gz-math7::gz-math7
  gz-common5::gz-common5
  gz-plugin2::gz-plugin2
)

# Install the plugin library where gz can find it.
install(TARGETS LivoxLidarSystem
  LIBRARY DESTINATION lib
)

# Install scan pattern CSVs.
install(DIRECTORY scan_mode/
  DESTINATION share/${PROJECT_NAME}/scan_mode
)

ament_package()
