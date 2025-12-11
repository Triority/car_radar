// Copyright (c) 2023-2024 Sensrad

#pragma once

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <oden_interfaces/msg/free_space.hpp>
#include <oden_interfaces/msg/ground_plane.hpp>
#include <oden_interfaces/msg/multi_object_tracking.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace perception_visualizer {

class PerceptionVisualizer : public rclcpp::Node {
public:
  PerceptionVisualizer();

  ~PerceptionVisualizer() {}

private:
  // Quality of service queue length
  constexpr static int QOS_BACKLOG_ = 10;

  // Subscriptions
  rclcpp::Subscription<oden_interfaces::msg::GroundPlane>::SharedPtr
      ground_plane_data_subscription_;

  rclcpp::Subscription<oden_interfaces::msg::MultiObjectTracking>::SharedPtr
      object_tracks_subscription_;

  // Sync subscriptions with message_filters
  // Note that it's necessary to subscribe ground plane data msg twice because
  // these two subscriptions are in different types, we do so to ensure that
  // ground plane data callback can be called independent from free space data
  // msg so that ground plane polygon can still be available even free space
  // data is lost
  message_filters::Subscriber<oden_interfaces::msg::GroundPlane>
      sync_ground_plane_data_subscription_;

  message_filters::Subscriber<oden_interfaces::msg::FreeSpace>
      sync_free_space_data_subscription_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      ground_plane_polygon_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      free_space_marker_publisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      aabb_object_tracks_publisher_;

  // Time synchronizer
  std::shared_ptr<message_filters::TimeSynchronizer<
      oden_interfaces::msg::GroundPlane, oden_interfaces::msg::FreeSpace>>
      sync_groundplane_freespace_;

  // Callback functions
  void groundPlaneDataCallback(
      const oden_interfaces::msg::GroundPlane::ConstSharedPtr
          &ground_plane_data);

  void syncGroundPlaneFreeSpaceCallback(
      const oden_interfaces::msg::GroundPlane::ConstSharedPtr
          &ground_plane_data,
      const oden_interfaces::msg::FreeSpace::ConstSharedPtr &free_space_data);

  void objectTracksCallback(
      const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
          &object_tracks_data);
};

} // namespace perception_visualizer
