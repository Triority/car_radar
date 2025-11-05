// Copyright (c) Sensrad 2024

#pragma once

// There is a problem in Eigen 3.4.0: it generates a warning in
// TriangularMatrixVector.h:332:12: warning: ‘result’ may be used uninitialized
// [-Wmaybe-uninitialized]
//
// https://github.com/PDAL/PDAL/issues/4102
//
// The GCC command below suppresses this warning.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace common {

constexpr int CLUSTER_UNDEFINED = -2;
constexpr int CLUSTER_NOISE = -1;

enum MotionStatus : uint8_t { STATIONARY = 0, UNKNOWN = 1, DYNAMIC = 2 };

enum Type : uint8_t {
  CLASS_UNKNOWN = 0,
  CLASS_PEDESTRIAN = 1,
  CLASS_BICYCLE = 2,
  CLASS_CAR = 3,
  CLASS_UTILITY_VEHICLE = 4,
  CLASS_TRUCK = 5,
  NUM_CLASSES = 6
};

// Definition of track state
enum TrackState : int {
  INITIALIZING = 0,
  TENTATIVE = 1,
  MATURE = 2,
  TERMINATE = 3
};

// Definition of perception compute status
enum Status : int {
  NORMAL = 0,
  LARGE_TIME_GAP = 1,
  SHORT_TIME_GAP = 2,
  NEGATIVE_TIME_GAP = 3
};

// Disable clang tidy checks on public visibility since we want these data
// carrying structs to directly accessible by the user.

// NOLINTBEGIN(misc-non-private-member-variables-in-classes,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
// Define format of input point-cloud
struct InputPoint {
  float range;
  float elevation;
  float azimuth;
  float power;
  float doppler;
  float x;
  float y;
  float z;
  std::size_t sample_index; // This is only for dealing with annotations

  InputPoint(float range, float elevation, float azimuth, float power,
             float doppler, float x, float y, float z, std::size_t sample_index)
      : range(range), elevation(elevation), azimuth(azimuth), power(power),
        doppler(doppler), x(x), y(y), z(z), sample_index{sample_index} {}
};
typedef std::vector<InputPoint> IncomingPointCloud;

// Define format of point-cloud data sent out from perception. Due to
// limitations in pybind11 we cannot easily inherit from InputPoint, that's
// we cannot reuse "InputPoint" here.
struct OutputPoint {
  float range;
  float elevation;
  float azimuth;
  float power;
  float doppler;
  float x;
  float y;
  float z;
  std::size_t sample_index; // This is only for dealing with annotations
  float disturbance_std;
  uint8_t motion_status;
  float delta_velocity;
  bool available_for_tracker;
  float distance_to_track;
  float track_speed;
  int cluster_idx;
  float distance_to_ground_plane;
  uint8_t multi_path;

  OutputPoint(float range, float elevation, float azimuth, float power,
              float doppler, float x, float y, float z,
              std::size_t sample_index = 0, float disturbance_std = 0.0F,
              int8_t motion_status = common::MotionStatus::DYNAMIC,
              float delta_velocity = 0.0F, bool available_for_tracker = false,
              float distance_to_track = 1000.0F, float track_speed = 0.0F,
              int cluster_idx = common::CLUSTER_NOISE,
              float distance_to_ground_plane = 1000.0F, uint8_t multi_path = 0)
      : range(range), elevation(elevation), azimuth(azimuth), power(power),
        doppler(doppler), x(x), y(y), z(z), sample_index(sample_index),
        disturbance_std(disturbance_std), motion_status(motion_status),
        delta_velocity(delta_velocity),
        available_for_tracker(available_for_tracker),
        distance_to_track(distance_to_track), track_speed(track_speed),
        cluster_idx(cluster_idx),
        distance_to_ground_plane(distance_to_ground_plane),
        multi_path(multi_path) {}
};
typedef std::vector<OutputPoint> OutgoingPointCloud;

// Define format of detections created from clustered data.
struct Detection {
  float range;
  float radial_speed;
  float azimuth;
  float elevation;
  float power;
  float power_std;
  float doppler_std;
  float doppler;
  Eigen::Vector3f position;
  Eigen::Matrix3f covariance;
  int num_valid_detections;
  uint8_t type;
  float type_probability;
  int label_id;
  Detection()
      : range(0), radial_speed(0), azimuth(0), elevation(0), power(0),
        power_std(0), doppler_std(0), doppler(0), num_valid_detections(0),
        type(0), type_probability(0), label_id(0) {
    // Default values to ensure data is populated correctly
    covariance << 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F;
    position.setZero();
  }
};

typedef std::vector<Detection> Detections;

typedef std::unordered_map<int, std::vector<int>> cluster_map_t;

// Define output format of tracked objects
struct TrackedObject {
  size_t track_id;
  uint8_t track_state;
  float yaw;
  uint8_t type;
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  Eigen::Matrix3f extent;

  TrackedObject() : track_id(0), track_state(0), yaw(0.0F), type(0) {
    position.setZero();
    velocity.setZero();
    extent.setZero();
  }
};
typedef std::vector<TrackedObject> TrackedObjects;
} // namespace common

// NOLINTEND(misc-non-private-member-variables-in-classes,cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
