#include "floor_robot_cpp_python.hpp"
#include <rclcpp/rclcpp.hpp>

// ================================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto floor_robot_node = std::make_shared<FloorRobot>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(floor_robot_node);

  try
  {
    executor.spin();
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    executor.cancel();
    rclcpp::shutdown();
  }
}