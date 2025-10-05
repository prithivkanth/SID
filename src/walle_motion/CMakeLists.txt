cmake_minimum_required(VERSION 3.8)
project(walle_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(pd_motion_planner src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner ${dependencies})

add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit ${dependencies})


install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS 
  pd_motion_planner
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
