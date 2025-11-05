// Copyright (c) Sensrad 2023

#include "hugin_raf_pointcloud_parser_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  RCLCPP_INFO(rclcpp::get_logger("pointcloud_parser_node"),
              "Starting Hugin point cloud parser node...");

  rclcpp::init(argc, argv);

  auto pointcloud_parser = std::make_shared<RafPointcloudParserNode>();
  rclcpp::spin(pointcloud_parser);
  rclcpp::shutdown();

  return 0;
}
