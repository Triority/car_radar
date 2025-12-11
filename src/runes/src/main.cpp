// Copyright (c) Sensrad 2023

#include "PerceptionVisualizer.hpp"
#include "PolygonGenerator.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_visualizer::PerceptionVisualizer>());
  rclcpp::shutdown();

  return 0;
}
