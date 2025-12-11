// Copyright (c) 2023-2024 Sensrad

#include "PerceptionVisualizer.hpp"
#include "PolygonGenerator.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace perception_visualizer {

PerceptionVisualizer::PerceptionVisualizer() : Node("runes") {
  RCLCPP_INFO(get_logger(), "Runes node started");

  // Default values for incoming topics
  declare_parameter("frame_id", "radar_1");

  // Create subscriptions
  ground_plane_data_subscription_ =
      create_subscription<oden_interfaces::msg::GroundPlane>(
          "ground_plane_data", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::groundPlaneDataCallback, this, _1));

  sync_ground_plane_data_subscription_.subscribe(this, "ground_plane_data");

  sync_free_space_data_subscription_.subscribe(this, "free_space_data");

  object_tracks_subscription_ =
      create_subscription<oden_interfaces::msg::MultiObjectTracking>(
          "object_tracks", QOS_BACKLOG_,
          std::bind(&PerceptionVisualizer::objectTracksCallback, this, _1));

  // Create publishers
  ground_plane_polygon_publisher_ =
      create_publisher<geometry_msgs::msg::PolygonStamped>(
          "ground_plane_polygon", QOS_BACKLOG_);

  free_space_marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
          "free_space_marker", QOS_BACKLOG_);
  aabb_object_tracks_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
          "aabb_object_tracks", QOS_BACKLOG_);

  // Sync subscriptions
  sync_groundplane_freespace_ =
      std::make_shared<message_filters::TimeSynchronizer<
          oden_interfaces::msg::GroundPlane, oden_interfaces::msg::FreeSpace>>(
          sync_ground_plane_data_subscription_,
          sync_free_space_data_subscription_, QOS_BACKLOG_);

  sync_groundplane_freespace_->registerCallback(std::bind(
      &PerceptionVisualizer::syncGroundPlaneFreeSpaceCallback, this, _1, _2));
}

// Callback functions below
void PerceptionVisualizer::groundPlaneDataCallback(
    const oden_interfaces::msg::GroundPlane::ConstSharedPtr
        &ground_plane_data) {
  ground_plane_polygon_publisher_->publish(
      PolygonGenerator::getGroundPlanePolygon(ground_plane_data));
}

void PerceptionVisualizer::syncGroundPlaneFreeSpaceCallback(
    const oden_interfaces::msg::GroundPlane::ConstSharedPtr &ground_plane_data,
    const oden_interfaces::msg::FreeSpace::ConstSharedPtr &free_space_data) {
  free_space_marker_publisher_->publish(
      PolygonGenerator::getFreeSpaceMarker(ground_plane_data, free_space_data));
}

void PerceptionVisualizer::objectTracksCallback(
    const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
        &tracked_objects_data) {
  aabb_object_tracks_publisher_->publish(
      PolygonGenerator::getAABBObjectTracks(tracked_objects_data));
}

} // namespace perception_visualizer
