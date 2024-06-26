cmake_minimum_required(VERSION 3.8)
project(rwa3_group_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)

install(DIRECTORY
  launch
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

#--------------------------------
# Python
#--------------------------------

# install Python modules from first_package/first_package
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# install Python executables
install(PROGRAMS script/check_competition_state.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS script/orders.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS script/submit_order.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS script/ship_orders.py DESTINATION lib/${PROJECT_NAME})

ament_package()
