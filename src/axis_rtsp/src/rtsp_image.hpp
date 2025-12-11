// Copyright (c) Sensrad 2023

#pragma once

// ROS2
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
// Message types
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
// Published type
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
// Gstreamer headers
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
// STL
#include <mutex>
#include <thread>

class RTSPImage : public rclcpp::Node {
  const rclcpp::QoS qos_ = rclcpp::SystemDefaultsQoS();

  std::string output_format_;

  // Dimensions can only be set at startup.
  int width_;
  int height_;

  // glib main loop
  GMainLoop *main_loop_;
  // main loop thread
  std::thread main_loop_thread_;
  // mutex
  std::mutex pipeline_mutex_;

  // Output image
  sensor_msgs::msg::Image output_image_;
  sensor_msgs::msg::CompressedImage output_compressed_image_;
  // Camera info message
  sensor_msgs::msg::CameraInfo camera_info_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  // Compressed image publisher
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_image_publisher_;
  // Camera info publisher
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher;

  // Gstreamer elements
  GstElement *pipeline_ = nullptr;
  GstElement *sink_ = nullptr;

  // Struct for holding app sink callbacks
  GstAppSinkCallbacks callbacks_;

  // Static gstreamer C "trampoline" callbacks.
  static void eos_(GstAppSink *appsink, gpointer user_data);
  static GstFlowReturn new_preroll_(GstAppSink *appsink, gpointer user_data);
  static GstFlowReturn new_jpeg_sample_(GstAppSink *appsink,
                                        gpointer user_data);
  static GstFlowReturn new_rgb_sample_(GstAppSink *appsink, gpointer user_data);
  static gboolean new_event_(GstAppSink *appsink, gpointer user_data);
  static gboolean on_bus_message_(GstBus *bus, GstMessage *message,
                                  gpointer user_data);

  // Corresponding member functions.
  void eos(GstAppSink *appsink);
  GstFlowReturn new_preroll(GstAppSink *appsink);
  GstFlowReturn new_jpeg_sample(GstAppSink *appsink);
  GstFlowReturn new_rgb_sample(GstAppSink *appsink);
  bool new_event(GstAppSink *appsink);
  bool on_bus_message(GstBus *bus, GstMessage *message);

  // Creates gstreamer pipeline
  void create_pipeline(const std::string &uri, const std::string &user,
                       const std::string &password);

public:
  RTSPImage();
  virtual ~RTSPImage();
};
