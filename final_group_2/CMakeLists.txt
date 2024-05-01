cmake_minimum_required(VERSION 3.8)
project(final_group_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# include folder which has all the header files
include_directories(include)

# create a variable for all dependencies
set(MOVEIT_DEMO_INCLUDE_DEPENDS
  rclcpp 
  std_msgs
  ariac_msgs
  tf2_ros
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  geometric_shapes
  moveit_ros_planning_interface
  robot_commander_msgs
)

# find all dependencies
foreach(dependency IN ITEMS ${MOVEIT_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()


#-----------------------------
# C++
#-----------------------------



add_executable(competition_control_system_exe src/competition_control_system.cpp src/main_cpp.cpp)
ament_target_dependencies(competition_control_system_exe ${MOVEIT_DEMO_INCLUDE_DEPENDS})

install(TARGETS competition_control_system_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#-----------------------------
# Installation
#-----------------------------

install(TARGETS
competition_control_system_exe
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY 
  config
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
