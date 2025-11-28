#include "hugin_raf_control_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("control_node"),
              "Starting Hugin Control node...");

  rclcpp::init(argc, argv);

  auto control = std::make_shared<RafControlNode>();
  rclcpp::spin(control);
  rclcpp::shutdown();

  return 0;
}
