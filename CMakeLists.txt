cmake_minimum_required(VERSION 3.8)
project(p4b-follow-wall-gidrobotic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    geometry_msgs
    kobuki_ros_interfaces
    sensor_msgs
)

include_directories(include)
add_library(${PROJECT_NAME} SHARED
  src/move/controller.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(controllerNode src/controllerNode.cpp)
ament_target_dependencies(controllerNode ${dependencies})
target_link_libraries(controllerNode ${PROJECT_NAME})

install(TARGETS
  controllerNode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(${dependencies})

ament_package() 
