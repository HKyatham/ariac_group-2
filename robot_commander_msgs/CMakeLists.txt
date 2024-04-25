cmake_minimum_required(VERSION 3.8)
project(robot_commander_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(srv_files
  "srv/MoveTrayToAGV.srv"
  "srv/MoveRobotToTable.srv"
  "srv/MoveRobotToTray.srv"
  "srv/PickPartFromBin.srv"
  "srv/PlacePartOnTray.srv"
  "srv/EnterToolChanger.srv"
  "srv/ExitToolChanger.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}

  DEPENDENCIES
  std_msgs
  geometry_msgs
  ariac_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
