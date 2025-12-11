// Copyright (c) Sensrad 2023
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include <boost/format.hpp>

#include <liboden/IO_Types.hpp>
#include <liboden/PerceptionAPI.hpp>

#include "AnnotationPoint.hpp"

#include <oden_interfaces/msg/detections.hpp>
#include <oden_interfaces/msg/ego_motion.hpp>
#include <oden_interfaces/msg/free_space.hpp>
#include <oden_interfaces/msg/ground_plane.hpp>
#include <oden_interfaces/msg/multi_object_tracking.hpp>
#include <oden_interfaces/msg/object_track.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#pragma once

constexpr double NANOSEC_SCALE{1.0e9};
constexpr double MILLISEC_SCALE{1.0e3};

class RadarPerception : public rclcpp::Node {
public:
  RadarPerception();

  virtual ~RadarPerception() {
    RCLCPP_INFO(get_logger(), "Radar perception node stopped");
  }

private:
  static constexpr int QOS_BACKLOG = 5;

  // Number of ground plane coefficients
  static constexpr int NOF_PLANE_COEF = 4;

  // Indices for azimuth and range in free space matrix
  static constexpr int AZIMUTH_IDX = 0;
  static constexpr int RANGE_IDX = 1;

  // If pointcloud as larger than this, a reallocation will be done.
  static constexpr std::size_t POINTCLOUD_RESERVE = 8096;
  static constexpr std::size_t OBJECTS_RESERVE = 256;
  static constexpr std::size_t MAX_POINTS_TO_PROCESS = 7500;
  static constexpr std::size_t DETECTIONS_RESERVE = 256;
  static constexpr std::size_t TRACKED_OBJECTS_RESERVE = 256;
  static constexpr float MAX_ALLOWED_DISTURBANCE_STD = 0.05F;

  // Extended PointCloud, containing original data as well as derived
  // quantities
  common::OutgoingPointCloud outgoing_pointcloud_;
  common::IncomingPointCloud incoming_pointcloud_;
  AnnotationPointCloud annotated_pointcloud_;

  // Declare objects for publishing an extended point-cloud
  sensor_msgs::msg::PointCloud2 ext_pc_to_publish_;
  sensor_msgs::PointCloud2Modifier ext_pc_modifier_;

  // Perception object
  api::PerceptionAPI perception_;

  // Perception outputs
  common::Detections detections_;
  common::TrackedObjects tracked_objects_;

  // Objects for recording time of incoming radar point-cloud
  uint32_t radar_time_sec_;
  uint32_t radar_time_nanosec_;
  uint64_t radar_time_ms_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_subscription_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult
  on_parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      extended_pointcloud_publisher_;

  rclcpp::Publisher<oden_interfaces::msg::EgoMotion>::SharedPtr
      odometry_publisher_;

  rclcpp::Publisher<oden_interfaces::msg::GroundPlane>::SharedPtr
      groundplane_publisher_;

  rclcpp::Publisher<oden_interfaces::msg::FreeSpace>::SharedPtr
      free_space_publisher_;

  rclcpp::Publisher<oden_interfaces::msg::Detections>::SharedPtr
      detections_publisher_;

  rclcpp::Publisher<oden_interfaces::msg::MultiObjectTracking>::SharedPtr
      object_tracks_publisher_;

  void create_subscriptions();

  void radar_data_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2);

  void read_incoming_pointcloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2ct_list);

  void publish_extended_pointcloud();
  void publish_egomotion();
  void publish_groundplane();
  void publish_freespace();
  void publish_detections();
  void publish_tracked_objects();

  // Parameter reading service
  rclcpp::TimerBase::SharedPtr parameter_timer_;
  void read_params();
};
