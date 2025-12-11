
// Copyright (c) Sensrad 2023

#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>

#include "rtsp_image.hpp"

int main(int argc, char **argv) {

  // We need to initialize GStreamer before using it.
  // This will fail fatally on error.
  guint major, minor, micro, nano;
  gst_init(&argc, &argv);
  gst_version(&major, &minor, &micro, &nano);

  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // No node logger yet, so create a temporary one.
  RCLCPP_INFO(rclcpp::get_logger("rtsp_image"), "Using GStreamer: %d.%d.%d.%d",
              major, minor, micro, nano);

  // Create encoder node
  auto rtsp_image_node = std::make_shared<RTSPImage>();

  executor->add_node(rtsp_image_node);

  // Spin until shutdown.
  // The glib main loop is spun in a separate thread in the node.
  executor->spin();

  rclcpp::shutdown();

  return 0;
}
