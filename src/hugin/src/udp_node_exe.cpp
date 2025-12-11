// Copyright (c) Sensrad 2023
#include "hugin_udp_receiver_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("udp_receiver_node"),
              "Starting Hugin UDP node...");

  rclcpp::init(argc, argv);

  auto udp_receiver = std::make_shared<UdpReceiverNode>();
  rclcpp::spin(udp_receiver);
  rclcpp::shutdown();

  return 0;
}
