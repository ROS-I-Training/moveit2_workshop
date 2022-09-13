cmake_minimum_required(VERSION 3.8)
project(moveit2_workshop_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(app_simple 
                src/app_simple.cpp)
target_include_directories(app_simple 
                            PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
ament_target_dependencies(app_simple 
                            rclcpp 
                            moveit_ros_planning_interface)


add_executable(app_marker 
      src/app_marker.cpp)
target_include_directories(app_marker 
                            PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)
ament_target_dependencies(app_marker 
                            rclcpp 
                            moveit_ros_planning_interface)

install(TARGETS
  app_simple
  app_marker
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
