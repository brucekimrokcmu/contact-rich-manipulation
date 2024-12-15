cmake_minimum_required(VERSION 3.8)
project(planning)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the MoveIt2 install directory to CMAKE_PREFIX_PATH
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/external/install")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Add executables
add_executable(planning_node src/planning_node.cpp)

# Link libraries
target_link_libraries(planning_node
  ${rclcpp_LIBRARIES}
  moveit_core
  moveit_planners
)

# Install executables
install(TARGETS
  planning_node
  DESTINATION lib/${PROJECT_NAME}
)

# install headers
install(DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
