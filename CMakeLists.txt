cmake_minimum_required(VERSION 3.8)
project(icuas25_competition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(icuas25_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add C++ executable: territorial_waters
add_executable(territorial_waters src/territorial_waters.cpp)
ament_target_dependencies(territorial_waters
  rclcpp
  octomap
  octomap_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Add C++ executable: hotspots
add_executable(hotspots src/hotspots.cpp)
ament_target_dependencies(hotspots
  rclcpp
  octomap
  octomap_msgs
  visualization_msgs
  geometry_msgs
  Eigen3
  tf2
  tf2_ros
  icuas25_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install executáveis
install(TARGETS territorial_waters
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS hotspots
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/TransformWorld2Odom.py
  scripts/charging.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
