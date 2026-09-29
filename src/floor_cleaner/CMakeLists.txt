cmake_minimum_required(VERSION 3.8)
project(floor_cleaner)

# This package contains no C++ code: only launch files, robot description,
# world, parameters, RViz config and one Python node. ament_cmake just
# installs these into install/floor_cleaner/share/floor_cleaner.
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    worlds
    config
    rviz
    maps
  DESTINATION share/${PROJECT_NAME}
)

# Python node -> runnable with: ros2 run floor_cleaner cleaning_pattern_node.py
install(
  PROGRAMS
    scripts/cleaning_pattern_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
