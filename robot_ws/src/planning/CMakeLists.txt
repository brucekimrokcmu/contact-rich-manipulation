cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgs REQUIRED)
find_package(ompl REQUIRED)

# Include directories
include_directories(${OMPL_INCLUDE_DIRS}
  include
)

# Add motion_server executable
add_executable(motion_server src/motion_server.cpp src/simple_motion_planner.cpp)
target_include_directories(motion_server PUBLIC ${OMPL_INCLUDE_DIRS})
target_link_libraries(motion_server ${OMPL_LIBRARIES})
ament_target_dependencies(motion_server rclcpp msgs ompl)

# Add motion_client executable
add_executable(motion_client src/motion_client.cpp)
target_include_directories(motion_client PUBLIC ${OMPL_INCLUDE_DIRS})
target_link_libraries(motion_client ${OMPL_LIBRARIES})
ament_target_dependencies(motion_client rclcpp msgs ompl)

install(TARGETS
  motion_server
  motion_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
