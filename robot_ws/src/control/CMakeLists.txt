cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)



# add_executable(control_node src/control_node.cpp)  
# target_include_directories(control_node PRIVATE include)
# target_link_libraries(control_node Eigen3::Eigen)
# ament_target_dependencies(control_node rclcpp std_msgs geometry_msgs)
# install(TARGETS control_node
#   DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  # Abstract Interface Tests
  ament_add_gtest(test_control_input test/test_control_input.cpp)
  target_include_directories(test_control_input PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(test_control_input Eigen3::Eigen)

  ament_add_gtest(test_state test/test_state.cpp)
  target_include_directories(test_state PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(test_state Eigen3::Eigen)

  ament_add_gtest(test_dynamics test/test_dynamics.cpp)
  target_include_directories(test_dynamics PRIVATE include ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(test_dynamics Eigen3::Eigen)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()
