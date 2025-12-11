// Copyright (c) Sensrad 2023-2024
#pragma once
#include <memory>
#include <optional>

#include "IO_Types.hpp"

namespace api {
class PerceptionAPI {
private:
  class PerceptionImpl;

  std::unique_ptr<PerceptionImpl> pimpl_;

public:
  explicit PerceptionAPI(size_t max_nof_points);

  // Delete unused constructors
  PerceptionAPI(const PerceptionAPI &) = delete;
  PerceptionAPI &operator=(const PerceptionAPI &) = delete;
  PerceptionAPI(PerceptionAPI &&) = delete;
  PerceptionAPI &operator=(PerceptionAPI &&) = delete;

  virtual ~PerceptionAPI();

  // API functions

  // Main update function for processing a point-cloud
  common::OutgoingPointCloud
  update(const common::IncomingPointCloud &incoming_pointcloud,
         uint64_t timestamp_ms);

  // Velocity related data
  std::optional<Eigen::Vector3f> vel_xyz_radar();
  bool vel_is_available();

  // This function sets the dynamic mode of the velocity estimator. If
  // dynamic is false, it is assumed that the radar operates in a stationary
  // environment.
  void set_velocity_mode(bool dynamic);

  // Ground plane related data
  std::optional<Eigen::Vector4f> groundPlane();
  bool groundPlaneAvailable();

  // This function sets dynamic mode of the ground plane estimator.
  void setGroundPlaneMode(bool dynamic);
  void setVerticalHeight(float vertical_height);
  void setElevationAngle(float radar_elevation_angle);

  // Retrive the detections produced after clustering
  common::Detections get_detections();
  int number_of_clusters();

  // Retrive the objects in an implementation independent way
  void updateTrackedObjects(common::TrackedObjects &tracked_objects);

  // Control whether disturbance suppression shall be run
  void suppress_disturbances(bool suppress);

  void activateLocalMax(bool activate);

  // Retrieve free space information
  const Eigen::MatrixXf &getFreeSpace();

  // Retrieve the status of the perception module
  common::Status getStatus(void);
};
} // namespace api
