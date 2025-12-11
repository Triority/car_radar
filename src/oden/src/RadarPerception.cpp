// Copyright (c) Sensrad 2023-2024

#include <boost/format.hpp>
#include <chrono>

#include "AnnotationPoint.hpp"
#include "RadarPerception.hpp"

using PointField = sensor_msgs::msg::PointField;
using PointCloud2ConstIterator = sensor_msgs::PointCloud2ConstIterator<float>;
using PointCloud2Iterator = sensor_msgs::PointCloud2Iterator<float>;
using PointCloud2IteratorINT16 = sensor_msgs::PointCloud2Iterator<int16_t>;
using PointCloud2IteratorUINT8 = sensor_msgs::PointCloud2Iterator<uint8_t>;
using std::placeholders::_1;

constexpr static int QOS_BACKLOG = 10; // QoS backlog size
rcl_interfaces::msg::SetParametersResult
RadarPerception::on_parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;

  RCLCPP_INFO(get_logger(), "Parameters changed");

  for (const auto &parameter : parameters) {
    RCLCPP_INFO(get_logger(), "Parameters changed: %s",
                parameter.get_name().c_str());

    if (parameter.get_name() == "dynamic") {
      const auto current_mode = parameter.as_bool();
      perception_.set_velocity_mode(current_mode);
      perception_.setGroundPlaneMode(current_mode);
      result.reason = (boost::format("dynamic = %d") % current_mode).str();
      result.successful = true;
      RCLCPP_INFO(get_logger(), " %d", current_mode);
    } else if (parameter.get_name() == "suppress_disturbances") {
      const auto current_mode = parameter.as_bool();
      result.reason =
          (boost::format("suppress_disturbances = %d") % current_mode).str();
      result.successful = true;
      RCLCPP_INFO(get_logger(), " %d", current_mode);
    } else if (parameter.get_name() == "local_max") {
      const auto local_max = parameter.as_bool();
      perception_.activateLocalMax(local_max);
      result.reason = (boost::format("local_max = %d") % local_max).str();
      result.successful = true;
      RCLCPP_INFO(get_logger(), " %d", local_max);
    } else if (parameter.get_name() == "vertical_height") {
      const auto vertical_height = parameter.get_value<float>();
      perception_.setVerticalHeight(vertical_height);
      result.reason =
          (boost::format("vertical_height = %d") % vertical_height).str();
      result.successful = true;
    } else if (parameter.get_name() == "radar_elevation_angle") {
      const auto radar_elevation_angle = parameter.get_value<float>();
      perception_.setElevationAngle(radar_elevation_angle);
      result.reason =
          (boost::format("radar_elevation_angle = %d") % radar_elevation_angle)
              .str();
      result.successful = true;
    } else {
      result.successful = false;
      result.reason = "invalid_parameter";
    }
  }

  // Here update class attributes, do some actions, etc.
  return result;
}

// Function that grabs incoming point-cloud and creates
// internal representation
void RadarPerception::read_incoming_pointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2) {
  // Prepare iterators
  PointCloud2ConstIterator in_range_it(*pointcloud2, "range");
  PointCloud2ConstIterator in_elevation_it(*pointcloud2, "elevation");
  PointCloud2ConstIterator in_azimuth_it(*pointcloud2, "azimuth");
  PointCloud2ConstIterator in_power_it(*pointcloud2, "power");
  PointCloud2ConstIterator in_doppler_it(*pointcloud2, "doppler");
  PointCloud2ConstIterator in_x_it(*pointcloud2, "x");
  PointCloud2ConstIterator in_y_it(*pointcloud2, "y");
  PointCloud2ConstIterator in_z_it(*pointcloud2, "z");

  // Detect if annotation is available
  bool annotation_is_available = false;
  for (const auto &field : pointcloud2->fields) {
    if (field.name == "annotation_cluster_idx") {
      annotation_is_available = true;
    }
  }

  radar_time_sec_ = pointcloud2->header.stamp.sec;
  radar_time_nanosec_ = pointcloud2->header.stamp.nanosec;
  auto radar_time = static_cast<double>(radar_time_sec_) +
                    static_cast<double>(radar_time_nanosec_) / NANOSEC_SCALE;
  radar_time_ms_ = static_cast<uint64_t>(radar_time * MILLISEC_SCALE);

  RCLCPP_DEBUG(get_logger(), "Radar time (msec): %ld", radar_time_ms_);

  // Clear memory.
  incoming_pointcloud_.clear();
  outgoing_pointcloud_.clear();
  annotated_pointcloud_.clear();

  // Convert incoming PointCloud2 to internal format
  std::size_t ctr = 0;
  while (in_range_it != in_range_it.end()) {

    // Check for NaN
    bool is_nan = std::isnan(*in_range_it) || std::isnan(*in_elevation_it) ||
                  std::isnan(*in_azimuth_it) || std::isnan(*in_power_it) ||
                  std::isnan(*in_doppler_it) || std::isnan(*in_x_it) ||
                  std::isnan(*in_y_it) || std::isnan(*in_z_it);

    if (is_nan) {
      RCLCPP_WARN(get_logger(), "NaN in point-cloud, skipping point");
      ++in_range_it;
      ++in_elevation_it;
      ++in_azimuth_it;
      ++in_power_it;
      ++in_doppler_it;
      ++in_x_it;
      ++in_y_it;
      ++in_z_it;
      continue;
    }

    incoming_pointcloud_.emplace_back(
        *in_range_it, *in_elevation_it, *in_azimuth_it, *in_power_it,
        *in_doppler_it, *in_x_it, *in_y_it, *in_z_it, ctr);

    ++in_range_it;
    ++in_elevation_it;
    ++in_azimuth_it;
    ++in_power_it;
    ++in_doppler_it;
    ++in_x_it;
    ++in_y_it;
    ++in_z_it;
    ctr++;
  }

  // Sort point-cloud with respect to range; this should ideally not be
  // necessary but Arbe fails occasionally on this. Howver, this should be
  // fast since the point-cloud is "almost" sorted.
  std::sort(incoming_pointcloud_.begin(), incoming_pointcloud_.end(),
            [](const auto &a, const auto &b) { return a.range < b.range; });

  // Create annotated cloud. If not present in input, fill with default values
  if (annotation_is_available) {

    PointCloud2IteratorINT16 in_annotation_cluster_idx_it(
        *pointcloud2, "annotation_cluster_idx");

    PointCloud2IteratorUINT8 in_annotation_class_it(*pointcloud2,
                                                    "annotation_class");

    while (in_annotation_cluster_idx_it != in_annotation_cluster_idx_it.end()) {
      annotated_pointcloud_.emplace_back(*in_annotation_cluster_idx_it,
                                         *in_annotation_class_it);

      ++in_annotation_cluster_idx_it;
      ++in_annotation_class_it;
    }
  } else {
    const AnnotationPoint default_output{-1, 0};
    annotated_pointcloud_.clear();
    annotated_pointcloud_.resize(incoming_pointcloud_.size(), default_output);
  }
}

void RadarPerception::publish_detections() {
  detections_ = perception_.get_detections();

  oden_interfaces::msg::Detections pub_det;

  pub_det.header.stamp.sec = radar_time_sec_;
  pub_det.header.stamp.nanosec = radar_time_nanosec_;
  pub_det.header.frame_id = this->get_parameter("frame_id").as_string();

  // Loop over incoming detections
  for (const auto &det_i : detections_) {
    oden_interfaces::msg::Detection this_det;

    this_det.range = det_i.range;
    this_det.azimuth = det_i.azimuth;
    this_det.elevation = det_i.elevation;
    this_det.radial_speed = det_i.radial_speed;
    this_det.power = det_i.power;
    this_det.power_std = det_i.power_std;
    this_det.doppler_std = det_i.doppler_std;
    this_det.position[0] = det_i.position(0);
    this_det.position[1] = det_i.position(1);
    this_det.position[2] = det_i.position(2);
    // Using row major order
    this_det.covariance[0] = det_i.covariance(0, 0);
    this_det.covariance[1] = det_i.covariance(0, 1);
    this_det.covariance[2] = det_i.covariance(0, 2);
    this_det.covariance[3] = det_i.covariance(1, 0);
    this_det.covariance[4] = det_i.covariance(1, 1);
    this_det.covariance[5] = det_i.covariance(1, 2);
    this_det.covariance[6] = det_i.covariance(2, 0);
    this_det.covariance[7] = det_i.covariance(2, 1);
    this_det.covariance[8] = det_i.covariance(2, 2);
    this_det.type = det_i.type;
    this_det.type_probability = det_i.type_probability;
    this_det.label_id = det_i.label_id;
    this_det.num_valid_detections = det_i.num_valid_detections;

    pub_det.detection_list.push_back(this_det);
  }

  detections_publisher_->publish(pub_det);
}

void RadarPerception::publish_tracked_objects() {
  // Defined a message for object track list

  // Grab objects
  perception_.updateTrackedObjects(tracked_objects_);

  // Port the objects to the published message
  oden_interfaces::msg::MultiObjectTracking object_tracks;

  object_tracks.header.stamp.sec = radar_time_sec_;
  object_tracks.header.stamp.nanosec = radar_time_nanosec_;
  object_tracks.header.frame_id = this->get_parameter("frame_id").as_string();

  for (const auto &track : tracked_objects_) {
    oden_interfaces::msg::ObjectTrack single_track;
    single_track.track_id = track.track_id;
    single_track.track_state = track.track_state;
    single_track.yaw = track.yaw;
    single_track.type = track.type;

    for (int dimension = 0; dimension < 3; ++dimension) {
      single_track.position[dimension] = track.position(dimension);
      single_track.velocity[dimension] = track.velocity(dimension);
    }

    // Using row major order
    single_track.extent[0] = track.extent(0, 0);
    single_track.extent[1] = track.extent(0, 1);
    single_track.extent[2] = track.extent(0, 2);
    single_track.extent[3] = track.extent(1, 0);
    single_track.extent[4] = track.extent(1, 1);
    single_track.extent[5] = track.extent(1, 2);
    single_track.extent[6] = track.extent(2, 0);
    single_track.extent[7] = track.extent(2, 1);
    single_track.extent[8] = track.extent(2, 2);

    object_tracks.object_track_list.push_back(single_track);
  }

  object_tracks_publisher_->publish(object_tracks);
}

void RadarPerception::publish_egomotion() {

  oden_interfaces::msg::EgoMotion ego_motion;

  ego_motion.header.stamp.sec = radar_time_sec_;
  ego_motion.header.stamp.nanosec = radar_time_nanosec_;
  ego_motion.header.frame_id = this->get_parameter("frame_id").as_string();

  if (auto vel_xyz = perception_.vel_xyz_radar()) {
    ego_motion.is_valid = perception_.vel_is_available();
    ego_motion.vel_x = vel_xyz->x();
    ego_motion.vel_y = vel_xyz->y();
    ego_motion.vel_z = vel_xyz->x();

  } else {
    ego_motion.is_valid = false;
    ego_motion.vel_x = 0.0f;
    ego_motion.vel_y = 0.0f;
    ego_motion.vel_z = 0.0f;
  }
  odometry_publisher_->publish(ego_motion);
}

void RadarPerception::publish_groundplane() {
  // Define a message for ground plane data
  oden_interfaces::msg::GroundPlane ground_plane_data;

  ground_plane_data.header.stamp.sec = radar_time_sec_;
  ground_plane_data.header.stamp.nanosec = radar_time_nanosec_;
  ground_plane_data.header.frame_id = get_parameter("frame_id").as_string();

  ground_plane_data.is_valid = perception_.groundPlaneAvailable();
  if (ground_plane_data.is_valid) {
    // Extract plane coefficients
    // Get the ground plane coefficients
    const auto plane_coef = perception_.groundPlane().value();

    for (int i = 0; i < NOF_PLANE_COEF; ++i) {
      ground_plane_data.plane.coef[i] = plane_coef[i];
    }
  }

  // Publish the message
  groundplane_publisher_->publish(ground_plane_data);
}

void RadarPerception::publish_freespace() {
  // Define a message for free space data
  oden_interfaces::msg::FreeSpace free_space_data;

  free_space_data.header.stamp.sec = radar_time_sec_;
  free_space_data.header.stamp.nanosec = radar_time_nanosec_;
  free_space_data.header.frame_id = get_parameter("frame_id").as_string();

  free_space_data.is_valid = perception_.groundPlaneAvailable();
  if (free_space_data.is_valid) {
    // Get the free space
    const Eigen::MatrixXf &free_space = perception_.getFreeSpace();
    // Fill in the message
    free_space_data.rows = free_space.rows();

    free_space_data.azimuth.resize(free_space_data.rows);
    free_space_data.range.resize(free_space_data.rows);

    Eigen::Map<Eigen::MatrixXf>(free_space_data.azimuth.data(),
                                free_space_data.rows, 1) =
        free_space.col(AZIMUTH_IDX);
    Eigen::Map<Eigen::MatrixXf>(free_space_data.range.data(),
                                free_space_data.rows, 1) =
        free_space.col(RANGE_IDX);
  }

  // Publish the message
  free_space_publisher_->publish(free_space_data);
}

// Publish extended point cloud
void RadarPerception::publish_extended_pointcloud() {
  ext_pc_modifier_.clear();

  std::size_t nof_points{outgoing_pointcloud_.size()};

  ext_pc_modifier_.resize(nof_points);
  ext_pc_to_publish_.width = static_cast<uint32_t>(nof_points);
  ext_pc_to_publish_.height = 1;

  ext_pc_to_publish_.header = std_msgs::msg::Header();
  ext_pc_to_publish_.header.frame_id =
      this->get_parameter("frame_id").as_string();

  ext_pc_to_publish_.header.stamp.sec = radar_time_sec_;
  ext_pc_to_publish_.header.stamp.nanosec = radar_time_nanosec_;

  PointCloud2Iterator out_range_it(ext_pc_to_publish_, "range");
  PointCloud2Iterator out_power_it(ext_pc_to_publish_, "power");
  PointCloud2Iterator out_doppler_it(ext_pc_to_publish_, "doppler");
  PointCloud2Iterator out_x_it(ext_pc_to_publish_, "x");
  PointCloud2Iterator out_y_it(ext_pc_to_publish_, "y");
  PointCloud2Iterator out_z_it(ext_pc_to_publish_, "z");
  PointCloud2Iterator out_delta_velocity_it(ext_pc_to_publish_,
                                            "delta_velocity");
  PointCloud2IteratorUINT8 out_motion_status_it(ext_pc_to_publish_,
                                                "motion_status");
  PointCloud2IteratorUINT8 out_available_for_tracker_it(
      ext_pc_to_publish_, "available_for_tracker");
  PointCloud2IteratorINT16 out_cluster_idx_it(ext_pc_to_publish_,
                                              "cluster_idx");
  PointCloud2IteratorINT16 out_annotation_cluster_idx_it(
      ext_pc_to_publish_, "annotation_cluster_idx");

  PointCloud2IteratorUINT8 out_annotation_class_it(ext_pc_to_publish_,
                                                   "annotation_class");
  PointCloud2Iterator out_distance_to_ground_plane(ext_pc_to_publish_,
                                                   "distance_to_ground_plane");
  PointCloud2Iterator out_dist_it(ext_pc_to_publish_, "disturbance_std");

  PointCloud2IteratorUINT8 out_multi_path_it(ext_pc_to_publish_, "multi_path");

  // Create outgoing extended point cloud
  for (size_t i = 0; i < nof_points; ++i) {

    *out_range_it = outgoing_pointcloud_[i].range;
    *out_power_it = outgoing_pointcloud_[i].power;
    *out_doppler_it = outgoing_pointcloud_[i].doppler;
    *out_x_it = outgoing_pointcloud_[i].x;
    *out_y_it = outgoing_pointcloud_[i].y;
    *out_z_it = outgoing_pointcloud_[i].z;
    *out_delta_velocity_it = outgoing_pointcloud_[i].delta_velocity;
    *out_motion_status_it = outgoing_pointcloud_[i].motion_status;
    *out_available_for_tracker_it =
        outgoing_pointcloud_[i].available_for_tracker;
    *out_cluster_idx_it = outgoing_pointcloud_[i].cluster_idx;

    const auto sample_idx = outgoing_pointcloud_[i].sample_index;
    *out_annotation_cluster_idx_it =
        annotated_pointcloud_[sample_idx].annotation_cluster_idx;
    *out_annotation_class_it =
        annotated_pointcloud_[sample_idx].annotation_class;
    *out_distance_to_ground_plane =
        outgoing_pointcloud_[i].distance_to_ground_plane;
    *out_dist_it = outgoing_pointcloud_[i].disturbance_std;
    *out_multi_path_it = outgoing_pointcloud_[i].multi_path;
    ++out_range_it;
    ++out_power_it;
    ++out_doppler_it;
    ++out_x_it;
    ++out_y_it;
    ++out_z_it;
    ++out_delta_velocity_it;
    ++out_motion_status_it;
    ++out_available_for_tracker_it;
    ++out_cluster_idx_it;
    ++out_annotation_cluster_idx_it;
    ++out_annotation_class_it;
    ++out_distance_to_ground_plane;
    ++out_dist_it;
    ++out_multi_path_it;
  }
  extended_pointcloud_publisher_->publish(ext_pc_to_publish_);
}

// Callback function for receiving point clouds from the radar
void RadarPerception::radar_data_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud2) {

  auto t1 = std::chrono::high_resolution_clock::now();
  read_incoming_pointcloud(pointcloud2);

  // Run perception
  outgoing_pointcloud_ =
      perception_.update(incoming_pointcloud_, radar_time_ms_);

  // Publish the results
  publish_egomotion();
  publish_groundplane();
  publish_freespace();
  publish_detections();
  publish_tracked_objects();
  publish_extended_pointcloud();

  // For debug and profiling
  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> ms_double = t2 - t1;

  RCLCPP_DEBUG(get_logger(), "Received pointcloud with %d points",
               pointcloud2->width * pointcloud2->height);

  RCLCPP_DEBUG(get_logger(), "Execution time: %f (ms)",
               static_cast<double>(ms_double.count()));
}

void RadarPerception::create_subscriptions() {
  pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      this->get_parameter("radar_topic").as_string(), QOS_BACKLOG,
      std::bind(&RadarPerception::radar_data_callback, this, _1));

  std::string tmp = this->get_parameter("radar_topic").as_string();
  RCLCPP_INFO(get_logger(), "Using data from topic: %s", tmp.c_str());
}

// Constructor for object running Sensrad radar perception
RadarPerception::RadarPerception()
    : Node("oden"), ext_pc_modifier_{ext_pc_to_publish_},
      perception_{MAX_POINTS_TO_PROCESS} {

  // Default values for incoming topics
  declare_parameter("radar_topic", "/hugin_raf_1/radar_data");
  declare_parameter("tracker_topic", "object_list");
  declare_parameter("frame_id", "radar_1");

  // Default values for radar perception
  const bool dynamic = declare_parameter("dynamic", true);
  const bool suppress_disturbances =
      declare_parameter("suppress_disturbances", true);
  const bool local_max = declare_parameter("local_max", true);
  const float vertical_height = declare_parameter("vertical_height", 1.0f);
  const float radar_elevation_angle =
      declare_parameter("radar_elevation_angle", 0.0f);

  perception_.set_velocity_mode(dynamic);
  perception_.setGroundPlaneMode(dynamic);
  perception_.suppress_disturbances(suppress_disturbances);
  perception_.activateLocalMax(local_max);
  perception_.setVerticalHeight(vertical_height);
  perception_.setElevationAngle(radar_elevation_angle);

  // Print configuration
  RCLCPP_INFO(get_logger(), "Oden configured with:");
  RCLCPP_INFO(get_logger(), "  dynamic: %d", dynamic);
  RCLCPP_INFO(get_logger(), "  suppress_disturbances: %d",
              suppress_disturbances);
  RCLCPP_INFO(get_logger(), "  local_max: %d", local_max);
  RCLCPP_INFO(get_logger(), "  vertical_height: %f", vertical_height);
  RCLCPP_INFO(get_logger(), "  radar_elevation_angle: %f",
              radar_elevation_angle);

  // Make space for pointclouds and objects
  incoming_pointcloud_.reserve(POINTCLOUD_RESERVE);
  outgoing_pointcloud_.reserve(POINTCLOUD_RESERVE);
  annotated_pointcloud_.reserve(POINTCLOUD_RESERVE);

  detections_.reserve(DETECTIONS_RESERVE);
  tracked_objects_.reserve(TRACKED_OBJECTS_RESERVE);

  radar_time_ms_ = 0;
  radar_time_sec_ = 0;
  radar_time_nanosec_ = 0;

  ext_pc_modifier_.setPointCloud2Fields(
      15, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1,
      PointField::FLOAT32, "power", 1, PointField::FLOAT32, "doppler", 1,
      PointField::FLOAT32, "delta_velocity", 1, PointField::FLOAT32,
      "motion_status", 1, PointField::UINT8, "available_for_tracker", 1,
      PointField::UINT8, "cluster_idx", 1, PointField::INT16,
      "annotation_cluster_idx", 1, PointField::INT16, "annotation_class", 1,
      PointField::UINT8, "distance_to_ground_plane", 1, PointField::FLOAT32,
      "disturbance_std", 1, PointField::FLOAT32, "range", 1,
      PointField::FLOAT32, "multi_path", 1, PointField::UINT8);

  create_subscriptions();

  // Create pointcloud publisher
  extended_pointcloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("extended_point_cloud",
                                                      QOS_BACKLOG);

  odometry_publisher_ = create_publisher<oden_interfaces::msg::EgoMotion>(
      "ego_motion", QOS_BACKLOG);

  groundplane_publisher_ = create_publisher<oden_interfaces::msg::GroundPlane>(
      "ground_plane_data", QOS_BACKLOG);

  free_space_publisher_ = create_publisher<oden_interfaces::msg::FreeSpace>(
      "free_space_data", QOS_BACKLOG);

  detections_publisher_ = create_publisher<oden_interfaces::msg::Detections>(
      "detections", QOS_BACKLOG);
  object_tracks_publisher_ =
      create_publisher<oden_interfaces::msg::MultiObjectTracking>(
          "object_tracks", QOS_BACKLOG);

  RCLCPP_INFO(get_logger(), "Oden node started");

  // Launch service for changing parameters
  param_callback_handle_ = add_on_set_parameters_callback(std::bind(
      &RadarPerception::on_parameters_callback, this, std::placeholders::_1));
};
