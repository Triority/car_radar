// Copyright (c) 2024 Sensrad

#pragma once
#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <oden_interfaces/msg/multi_object_tracking.hpp>

#include "PerceptionVisualizer.hpp"
#include <liboden/IO_Types.hpp>
namespace perception_visualizer {

class PolygonGenerator {
public:
  PolygonGenerator() {}

  ~PolygonGenerator() {}

  // Populate vertices in a polygon
  static void populateVertices(geometry_msgs::msg::Polygon &polygon,
                               const Eigen::Vector3f &point);

  // Generate ground plane polygon given data message
  static geometry_msgs::msg::PolygonStamped
  getGroundPlanePolygon(const oden_interfaces::msg::GroundPlane::ConstSharedPtr
                            &ground_plane_data);

  // Generate free space polygon given data message
  static visualization_msgs::msg::MarkerArray getFreeSpaceMarker(
      const oden_interfaces::msg::GroundPlane::ConstSharedPtr
          &ground_plane_data,
      const oden_interfaces::msg::FreeSpace::ConstSharedPtr &free_space_data);

  // Generate AABB object tracks given data message
  static visualization_msgs::msg::MarkerArray getAABBObjectTracks(
      const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
          &tracked_objects_data);

private:
  // Tolerance for judging if a value is zero
  static constexpr double ZERO_TOLERANCE = 1e-7;

  // Scale constant for ground plane polygon
  static constexpr float GROUND_PLANE_SCALE = 5.0f;

  // Coordinate dimension index for a point
  static constexpr int POINT_DIM_X = 0;
  static constexpr int POINT_DIM_Y = 1;
  static constexpr int POINT_DIM_Z = 2;

  // Coefficient index for a plane
  static constexpr int PLANE_COEF_IDX_A = 0;
  static constexpr int PLANE_COEF_IDX_B = 1;
  static constexpr int PLANE_COEF_IDX_C = 2;
  static constexpr int PLANE_COEF_IDX_D = 3;

  // Parameters for AABB object tracks
  static constexpr float FADE_DURATION =
      0.1; // Duration in time for visibility of AABB
  static constexpr float NOF_STD_AABB = 1.5; // Number of standard deviations
                                             // for AABB size
  static constexpr float MIN_SIZE =
      0.1; // Protect agains zero size when computing square root
  static constexpr float LINE_WIDTH = 0.1; // Line width for AABB when drawing

  static constexpr double OPACITY = 1.0;

  struct Color {
    float a; // Alpha (Opacity)
    float r; // Red
    float g; // Green
    float b; // Blue
  };

  // Parameters for text marker over Bounding Box
  static constexpr float TEXT_OFFSET = 0.2;
  static constexpr float TEXT_SIZE = 0.6;

  static const std::unordered_map<common::Type, Color> colorMap_;

  static std::string typeToString(common::Type type);

  static float
  createAABBMarker(const oden_interfaces::msg::ObjectTrack &tracked_objects,
                   const std_msgs::msg::Header &header,
                   visualization_msgs::msg::Marker &aabb_object_track);

  static void
  createTextMarker(const oden_interfaces::msg::ObjectTrack &tracked_objects,
                   const std_msgs::msg::Header &header, float aabb_half_hight,
                   visualization_msgs::msg::Marker &aabb_object_text);
};
} // namespace perception_visualizer
