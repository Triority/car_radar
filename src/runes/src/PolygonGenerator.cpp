// Copyright (c) 2024 Sensrad

#include "PolygonGenerator.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_visualizer {

// Init of static member variables
const std::unordered_map<common::Type, PolygonGenerator::Color>
    PolygonGenerator::colorMap_ = {
        {common::Type::CLASS_UNKNOWN, {OPACITY, 0.5, 0.5, 0.5}},    // Grey
        {common::Type::CLASS_PEDESTRIAN, {OPACITY, 1.0, 0.0, 0.0}}, // Red
        {common::Type::CLASS_BICYCLE, {OPACITY, 0.0, 1.0, 0.0}},    // Green
        {common::Type::CLASS_CAR, {OPACITY, 0.0, 1.0, 1.0}},        // Cyan
        {common::Type::CLASS_UTILITY_VEHICLE, {OPACITY, 0.0, 0.0, 1.0}}, // Blue
        {common::Type::CLASS_TRUCK, {OPACITY, 1.0, 0.0, 1.0}} // Magenta
};

std::string PolygonGenerator::typeToString(common::Type type) {
  switch (type) {
  case common::Type::CLASS_UNKNOWN:
    return "UNKNOWN";
  case common::Type::CLASS_PEDESTRIAN:
    return "PEDESTRIAN";
  case common::Type::CLASS_BICYCLE:
    return "BICYCLE";
  case common::Type::CLASS_CAR:
    return "CAR";
  case common::Type::CLASS_UTILITY_VEHICLE:
    return "UTILITY_VEHICLE";
  case common::Type::CLASS_TRUCK:
    return "TRUCK";
  default:
    return "Invalid Type";
  }
};

// Populate vertices in a polygon
void PolygonGenerator::populateVertices(geometry_msgs::msg::Polygon &polygon,
                                        const Eigen::Vector3f &point) {
  // Function to add vertices to polygon
  geometry_msgs::msg::Point32 v;
  v.x = point[POINT_DIM_X];
  v.y = point[POINT_DIM_Y];
  v.z = point[POINT_DIM_Z];

  polygon.points.push_back(v);
}

// Generate ground plane polygon given data message
geometry_msgs::msg::PolygonStamped PolygonGenerator::getGroundPlanePolygon(
    const oden_interfaces::msg::GroundPlane::ConstSharedPtr
        &ground_plane_data) {
  // Define a message for visualization
  geometry_msgs::msg::PolygonStamped ground_plane_polygon;
  ground_plane_polygon.header = ground_plane_data->header;

  if (ground_plane_data->is_valid) {
    // Extract ground plane coefficients
    const float plane_coef_a = ground_plane_data->plane.coef[PLANE_COEF_IDX_A];
    const float plane_coef_b = ground_plane_data->plane.coef[PLANE_COEF_IDX_B];
    // Avoid division by zero
    const float plane_coef_c = std::max(
        ground_plane_data->plane.coef[PLANE_COEF_IDX_C], ZERO_TOLERANCE);
    const float plane_coef_d = ground_plane_data->plane.coef[PLANE_COEF_IDX_D];

    // Compute four vertices of the ground plane to define polygon
    Eigen::Vector3f point0(
        1.0f, 4.0f,
        (-plane_coef_a - plane_coef_b * 4.0f - plane_coef_d) / plane_coef_c);

    Eigen::Vector3f point1 = GROUND_PLANE_SCALE * point0;
    point1[POINT_DIM_Z] = (-plane_coef_a * point1[POINT_DIM_X] -
                           plane_coef_b * point1[POINT_DIM_Y] - plane_coef_d) /
                          plane_coef_c;

    Eigen::Vector3f point2 = GROUND_PLANE_SCALE * point0;
    point2[POINT_DIM_X] = -point2[POINT_DIM_X];
    point2[POINT_DIM_Z] = (-plane_coef_a * point2[POINT_DIM_X] -
                           plane_coef_b * point2[POINT_DIM_Y] - plane_coef_d) /
                          plane_coef_c;

    Eigen::Vector3f point3 = point0;
    point3[POINT_DIM_X] = -point3[POINT_DIM_X];
    point3[POINT_DIM_Z] = (-plane_coef_a * point3[POINT_DIM_X] -
                           plane_coef_b * point3[POINT_DIM_Y] - plane_coef_d) /
                          plane_coef_c;

    // Populate polygon with vertices
    populateVertices(ground_plane_polygon.polygon, point0);
    populateVertices(ground_plane_polygon.polygon, point1);
    populateVertices(ground_plane_polygon.polygon, point2);
    populateVertices(ground_plane_polygon.polygon, point3);
    populateVertices(ground_plane_polygon.polygon, point0);

  } else {
    // Set two vertices to 0,0,0 if ground plane is not available
    ground_plane_polygon.polygon.points.push_back(
        geometry_msgs::msg::Point32());
    ground_plane_polygon.polygon.points.push_back(
        geometry_msgs::msg::Point32());
  }

  return ground_plane_polygon;
}

// Generate free space area given data message
visualization_msgs::msg::MarkerArray PolygonGenerator::getFreeSpaceMarker(
    const oden_interfaces::msg::GroundPlane::ConstSharedPtr &ground_plane_data,
    const oden_interfaces::msg::FreeSpace::ConstSharedPtr &free_space_data) {

  visualization_msgs::msg::MarkerArray free_space_marker;

  // Define a message for area_marker
  visualization_msgs::msg::Marker area_marker;
  area_marker.header = free_space_data->header;
  area_marker.ns = "area_marker";
  area_marker.id = 238; // Unique unused ID for the marker
  area_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  area_marker.action = visualization_msgs::msg::Marker::ADD;
  area_marker.scale.x = 1;
  area_marker.scale.y = 1;
  area_marker.scale.z = 1;
  area_marker.color.a = 0.5;
  area_marker.color.r = 0.5;
  area_marker.color.g = 0.5;
  area_marker.color.b = 1.0;

  // Define a message for border_line_marker
  visualization_msgs::msg::Marker border_line_marker;
  border_line_marker.header = free_space_data->header;
  border_line_marker.ns = "border_line_marker";
  border_line_marker.id = 239; // Unique unused ID for the marker
  border_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  border_line_marker.action = visualization_msgs::msg::Marker::ADD;
  border_line_marker.scale.x = 0.1;
  border_line_marker.color.a = 1.0;
  border_line_marker.color.r = 0.3;
  border_line_marker.color.g = 0.3;
  border_line_marker.color.b = 1.0;

  if (free_space_data->is_valid) {
    // Get the ground plane coefficients
    const float plane_coef_a = ground_plane_data->plane.coef[PLANE_COEF_IDX_A];
    const float plane_coef_b = ground_plane_data->plane.coef[PLANE_COEF_IDX_B];
    // Avoid division by zero
    const float plane_coef_c = std::max(
        ground_plane_data->plane.coef[PLANE_COEF_IDX_C], ZERO_TOLERANCE);
    const float plane_coef_d = ground_plane_data->plane.coef[PLANE_COEF_IDX_D];

    geometry_msgs::msg::Point centerPoint;
    centerPoint.x = 0.0f;
    centerPoint.y = 0.0f;
    centerPoint.z = -plane_coef_d / plane_coef_c;
    border_line_marker.points.push_back(centerPoint);

    // Loop over the free_space matrix and populate the markers
    geometry_msgs::msg::Point currentPoint;
    geometry_msgs::msg::Point lastPoint;

    float curr_azimuth;
    float curr_range;
    for (unsigned int idx = 0; idx < free_space_data->rows; ++idx) {
      // Extract current azimuth and range values from free space matrix
      curr_azimuth = free_space_data->azimuth[idx];
      curr_range = free_space_data->range[idx];

      currentPoint.x = curr_range * std::sin(curr_azimuth);
      currentPoint.y = curr_range * std::cos(curr_azimuth);
      currentPoint.z = (-plane_coef_a * currentPoint.x -
                        plane_coef_b * currentPoint.y - plane_coef_d) /
                       plane_coef_c;

      if (idx > 0) {
        // Populate vertex for triangles
        area_marker.points.push_back(centerPoint);
        area_marker.points.push_back(currentPoint);
        area_marker.points.push_back(lastPoint);
      }
      // Populate vertex for border_line_marker
      border_line_marker.points.push_back(currentPoint);

      // Update lastPoint
      lastPoint = currentPoint;
    }
    // Populate the last vertex for border_line_marker
    border_line_marker.points.push_back(centerPoint);

  } else {
    // Set two vertices to 0,0,0 if free space is not available
    area_marker.points.push_back(geometry_msgs::msg::Point());
    area_marker.points.push_back(geometry_msgs::msg::Point());

    border_line_marker.points.push_back(geometry_msgs::msg::Point());
    border_line_marker.points.push_back(geometry_msgs::msg::Point());
  }
  // Populate the marker array
  free_space_marker.markers.push_back(area_marker);
  free_space_marker.markers.push_back(border_line_marker);

  return free_space_marker;
}

// Create a marker for the AABB (axis aligned bounding box) object
float PolygonGenerator::createAABBMarker(
    const oden_interfaces::msg::ObjectTrack &tracked_object,
    const std_msgs::msg::Header &header,
    visualization_msgs::msg::Marker &aabb_marker_object) {

  aabb_marker_object.header = header;
  aabb_marker_object.ns = "aabb_object_tracks";
  aabb_marker_object.id = tracked_object.track_id;
  aabb_marker_object.type = visualization_msgs::msg::Marker::LINE_STRIP;
  aabb_marker_object.action = visualization_msgs::msg::Marker::ADD;

  aabb_marker_object.pose.orientation.x = 0.0;
  aabb_marker_object.pose.orientation.y = 0.0;
  aabb_marker_object.pose.orientation.z = 0.0;
  aabb_marker_object.pose.orientation.w = 1.0;

  // Compute rotated extent matrix
  const float angle = tracked_object.yaw;
  Eigen::Matrix3f rotation_matrix;
  rotation_matrix << std::cos(angle), -std::sin(angle), 0, std::sin(angle),
      std::cos(angle), 0, 0, 0, 1;

  Eigen::Matrix3f extent;
  extent(0, 0) = tracked_object.extent[0];
  extent(0, 1) = tracked_object.extent[1];
  extent(0, 2) = tracked_object.extent[2];
  extent(1, 0) = tracked_object.extent[3];
  extent(1, 1) = tracked_object.extent[4];
  extent(1, 2) = tracked_object.extent[5];
  extent(2, 0) = tracked_object.extent[6];
  extent(2, 1) = tracked_object.extent[7];
  extent(2, 2) = tracked_object.extent[8];

  const Eigen::Matrix3f rotated_extent =
      rotation_matrix * extent * rotation_matrix.transpose();

  // Compute sizes of bounding box. This is a heuristic, would be
  // more accurate to compute eigenvalues of rotated_extent and use those.
  // But this is good enough for now.
  const float x_half_size =
      std::sqrt(std::max(rotated_extent(0, 0), MIN_SIZE)) * NOF_STD_AABB / 2.0F;
  const float y_half_size =
      std::sqrt(std::max(rotated_extent(1, 1), MIN_SIZE)) * NOF_STD_AABB / 2.0F;
  const float z_half_size =
      std::sqrt(std::max(rotated_extent(2, 2), MIN_SIZE)) * NOF_STD_AABB / 2.0F;

  // Compute points on the bounding box and add to the marker
  const Eigen::Vector3f p_center = {tracked_object.position[POINT_DIM_X],
                                    tracked_object.position[POINT_DIM_Y],
                                    tracked_object.position[POINT_DIM_Z]};

  const Eigen::Vector3f p_0 = {-x_half_size, -y_half_size, -z_half_size};
  const Eigen::Vector3f p_1 = {x_half_size, -y_half_size, -z_half_size};
  const Eigen::Vector3f p_2 = {x_half_size, -y_half_size, z_half_size};
  const Eigen::Vector3f p_3 = {-x_half_size, -y_half_size, z_half_size};
  const Eigen::Vector3f p_4 = {-x_half_size, y_half_size, -z_half_size};
  const Eigen::Vector3f p_5 = {x_half_size, y_half_size, -z_half_size};
  const Eigen::Vector3f p_6 = {x_half_size, y_half_size, z_half_size};
  const Eigen::Vector3f p_7 = {-x_half_size, y_half_size, z_half_size};
  const std::vector<Eigen::Vector3f> corner_points{
      p_0, p_3, p_7, p_4, p_0, p_1, p_5, p_4, p_7,
      p_6, p_5, p_1, p_2, p_6, p_2, p_3, p_0,
  };

  for (const auto &corner_point : corner_points) {
    geometry_msgs::msg::Point p_add;
    Eigen::Vector3f new_point =
        p_center + rotation_matrix.transpose() * corner_point;
    p_add.x = new_point[POINT_DIM_X];
    p_add.y = new_point[POINT_DIM_Y];
    p_add.z = new_point[POINT_DIM_Z];
    aabb_marker_object.points.push_back(p_add);
  }

  // Set the drawing width
  aabb_marker_object.scale.x = LINE_WIDTH;

  // Set color based on type, and cast to common::Type
  auto type = static_cast<common::Type>(tracked_object.type);
  auto this_color = colorMap_.at(common::Type::CLASS_UNKNOWN);
  if (colorMap_.find(type) != colorMap_.end()) {
    this_color = colorMap_.at(type);
  }
  aabb_marker_object.color.a = this_color.a;
  aabb_marker_object.color.r = this_color.r;
  aabb_marker_object.color.g = this_color.g;
  aabb_marker_object.color.b = this_color.b;
  aabb_marker_object.lifetime = rclcpp::Duration::from_seconds(FADE_DURATION);
  return z_half_size;
}

// Create a marker for the label text of the AABB (axis aligned bounding box)
// object
void PolygonGenerator::createTextMarker(
    const oden_interfaces::msg::ObjectTrack &tracked_object,
    const std_msgs::msg::Header &header, float aabb_half_hight,
    visualization_msgs::msg::Marker &text_marker_object) {

  const Eigen::Vector3f p_center = {tracked_object.position[POINT_DIM_X],
                                    tracked_object.position[POINT_DIM_Y],
                                    tracked_object.position[POINT_DIM_Z]};

  text_marker_object.header = header;
  text_marker_object.ns = "aabb_object_texts";
  text_marker_object.id = tracked_object.track_id;
  text_marker_object.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker_object.action = visualization_msgs::msg::Marker::ADD;
  text_marker_object.pose.position.x = p_center[POINT_DIM_X];
  text_marker_object.pose.position.y = p_center[POINT_DIM_Y];
  text_marker_object.pose.position.z =
      p_center[POINT_DIM_Z] + aabb_half_hight + TEXT_OFFSET;
  text_marker_object.scale.z = TEXT_SIZE;

  // Set color based on type, and cast to common::Type
  auto type = static_cast<common::Type>(tracked_object.type);
  if (type != common::Type::CLASS_UNKNOWN) {
    auto this_color = colorMap_.at(common::Type::CLASS_UNKNOWN);
    if (colorMap_.find(type) != colorMap_.end()) {
      this_color = colorMap_.at(type);
    }
    text_marker_object.color.a = this_color.a;
    text_marker_object.color.r = this_color.r;
    text_marker_object.color.g = this_color.g;
    text_marker_object.color.b = this_color.b;
  }
  text_marker_object.lifetime = rclcpp::Duration::from_seconds(FADE_DURATION);

  // Set the text content
  text_marker_object.text = PolygonGenerator::typeToString(type);
}

// Generate AABB (axis aligned bounding box) and text object for visualization
visualization_msgs::msg::MarkerArray PolygonGenerator::getAABBObjectTracks(
    const oden_interfaces::msg::MultiObjectTracking::ConstSharedPtr
        &tracked_objects) {
  // Define message for visualization
  // visualization_msgs::msg::MarkerArray marker_objects;
  visualization_msgs::msg::MarkerArray marker_objects;

  // Loop over the tracked objects and populate the marker array
  for (unsigned int idx = 0; idx < tracked_objects->object_track_list.size();
       ++idx) {
    const std_msgs::msg::Header &header = tracked_objects->header;
    auto &tracked_object = tracked_objects->object_track_list[idx];

    // Visualize only mature objects
    if (tracked_object.track_state == common::TrackState::MATURE) {
      // Create a new marker for the AABB
      visualization_msgs::msg::Marker aabb_marker_object;
      visualization_msgs::msg::Marker text_marker_object;

      float aabb_half_hight = PolygonGenerator::createAABBMarker(
          tracked_object, header, aabb_marker_object);
      PolygonGenerator::createTextMarker(tracked_object, header,
                                         aabb_half_hight, text_marker_object);

      // Populate the marker array
      marker_objects.markers.push_back(aabb_marker_object);
      marker_objects.markers.push_back(text_marker_object);
    }
  }
  return marker_objects;
}
} // namespace perception_visualizer
