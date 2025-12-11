// Copyright (c) Sensrad 2023-2024

#pragma once

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

// These files are auto generated from the .msg files in the msg folder
// They are named like this becuase of ROS2 conventions.
#include <raf_interfaces/msg/sequence_type.hpp>
#include <raf_interfaces/msg/t_point_cloud_v11.hpp>
#include <raf_interfaces/msg/udp_packet.hpp>

#include <arbe_raf/Raf_Api.h>
#include <arbe_raf/Utils.h>

#include <cstring>
#include <variant>

// Helper type for the visitor pattern
template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template <class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Helper function for checking of NaN
template <typename... Args> constexpr bool is_nan(Args const &...args) {
  return (... || std::isnan(args));
}

// Arbe point cloud packet layout
// The exact layout of this struct is important
struct __attribute__((__packed__)) RafPacket {
  TPointCloud_V1_1 pc11;                         // Header
  uint8_t data[1400 - sizeof(TPointCloud_V1_1)]; // Payload, 1400 - header bytes

  // Convenience function for accessing fields in header.
  uint64_t timestamp_ms() const noexcept {
    uint64_t time_ms = 0;

    time_ms = static_cast<uint64_t>(pc11.unTimeMsb) << 32;
    time_ms |= static_cast<uint64_t>(pc11.unTimeLsb);
    time_ms >>= 16;

    return time_ms;
  }

  rclcpp::Time timestamp() const noexcept {
    uint64_t time_ms = timestamp_ms();

    return rclcpp::Time(static_cast<int32_t>((time_ms / 1'000)),
                        static_cast<uint32_t>((time_ms % 1000) * 1'000'000),
                        RCL_SYSTEM_TIME);
  }

  std::uint16_t message_number() const noexcept { return pc11.usMessageNumber; }

  std::uint16_t frame_counter() const noexcept { return pc11.usFrameCounter; }

  bool is_last_packet() const noexcept { return pc11.ucLastPacket & 0x01; }
};

// State machine for parsing point-clouds.
// Possible states
namespace State {
struct Idle {};
struct In_Pointcloud {
  std::uint16_t frame_counter;
  std::uint16_t message_number;
};
struct Error {};
using V = std::variant<Idle, In_Pointcloud, Error>;
} // namespace State

// Possible events
namespace Event {
struct PointcloudPacket {
  RafPacket &packet;
};
struct Timeout {};

using V = std::variant<PointcloudPacket, Timeout>;
} // namespace Event

// Pointcloud parser node
class RafPointcloudParserNode : public rclcpp::Node {

  constexpr static int QOS_BACKLOG_INCOMING =
      300; // QoS backlog size for incoming UDP
  constexpr static int QOS_BACKLOG_OUTGOING =
      10; // QoS backlog size for outgoing pointcloud

  constexpr static int MAX_PACKET_TARGETS = 256; // Max targets per packet

  constexpr static size_t MAX_POINTCLOUD_SIZE =
      1 << 16; // Max total size of pointcloud in targets
  constexpr static size_t MAX_PACKET_SIZE = 1400;

  // Keep track of our current state
  State::V current_state_;

  // Buffers for current pointcloud in spherical and Cartesian coordinates.
  // Same information, different representation.
  std::vector<TargetGeneric> current_pointcloud_sph_;
  std::vector<TargetCartesian> current_pointcloud_cart_;

  // Last seen timestamp
  rclcpp::Time last_seen_timestamp_;

  // What to publish, are set via parameters.
  bool publish_pointcloud_;
  bool publish_header_;

  // Subscriber of udp packets
  rclcpp::Subscription<raf_interfaces::msg::UDPPacket>::SharedPtr
      udp_packet_subscription_;

  // Output pointcloud2.
  sensor_msgs::msg::PointCloud2 pointcloud_;
  sensor_msgs::PointCloud2Modifier pointcloud_modifier_;

  // PointCloud2 publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_publisher_;

  // Header meta data message
  raf_interfaces::msg::TPointCloudV11 pointcloud_header_;
  // Meta data publisher
  rclcpp::Publisher<raf_interfaces::msg::TPointCloudV11>::SharedPtr
      pointcloud_header_publisher_;

  // Converts current internal pointcloud to ROS2 pointcloud2 and
  // publishes it
  void publish_pointcloud() {

    // Assert that the two vectors are the same size
    if (current_pointcloud_sph_.size() != current_pointcloud_cart_.size()) {
      throw std::runtime_error("Internal error, pointcloud size mismatch");
    }

    pointcloud_modifier_.clear();

    // Resize pointcloud2 to current pointcloud size
    pointcloud_modifier_.resize(current_pointcloud_sph_.size());
    pointcloud_.width = static_cast<uint32_t>(current_pointcloud_sph_.size());
    pointcloud_.height = 1;

    // Set header timestamp
    pointcloud_.header.stamp = last_seen_timestamp_;

    // Get pointcloud2 iterators
    using PointCloud2Iterator = sensor_msgs::PointCloud2Iterator<float>;
    PointCloud2Iterator iter_x(pointcloud_, "x"), iter_y(pointcloud_, "y"),
        iter_z(pointcloud_, "z"), iter_r(pointcloud_, "range"),
        iter_el(pointcloud_, "elevation"), iter_az(pointcloud_, "azimuth"),
        iter_power(pointcloud_, "power"), iter_doppler(pointcloud_, "doppler");

    auto targets = current_pointcloud_sph_.begin();
    auto cartesian = current_pointcloud_cart_.cbegin();

    // Count number of valid points
    size_t num_valid_points = 0;
    for (size_t i = 0; i < current_pointcloud_sph_.size(); i++) {

      // Skip NaN points
      if (is_nan(current_pointcloud_sph_[i].range,
                 current_pointcloud_sph_[i].elevation,
                 current_pointcloud_sph_[i].azimuth,
                 current_pointcloud_cart_[i].x, current_pointcloud_cart_[i].y,
                 current_pointcloud_cart_[i].z)) {
        ++targets;
        ++cartesian;
        // Skip this point
        continue;
      }
      num_valid_points++;

      // Assign cartesian coordinates
      *iter_x = static_cast<float>(cartesian->x);
      *iter_y = static_cast<float>(cartesian->y);
      *iter_z = static_cast<float>(cartesian->z);

      // Assign spherical coordinates
      *iter_r = targets->range;
      *iter_el = targets->elevation;
      *iter_az = targets->azimuth;
      *iter_power = targets->power;
      *iter_doppler = targets->doppler;

      // Advance destination iterators
      ++iter_x;
      ++iter_y;
      ++iter_z;

      ++iter_r;
      ++iter_el;
      ++iter_az;
      ++iter_power;
      ++iter_doppler;

      // Advance source iterators
      ++targets;
      ++cartesian;
    }

    // Resize pointcloud2 to number of valid points if NaN points were found
    if (num_valid_points != current_pointcloud_sph_.size()) {
      pointcloud_modifier_.resize(num_valid_points);
      pointcloud_.width = static_cast<uint32_t>(num_valid_points);
    }

    // Publish pointcloud
    pointcloud_publisher_->publish(pointcloud_);

    // Clear after publishing.
    clear_pointcloud();
  }

  void publish_header_from_packet(const RafPacket &packet) {
    // Set header timestamp
    pointcloud_header_.header.stamp = packet.timestamp();

    // Set header fields
    pointcloud_header_.length = packet.pc11.unLength;
    pointcloud_header_.frame_counter = packet.pc11.usFrameCounter;
    pointcloud_header_.message_number = packet.pc11.usMessageNumber;
    pointcloud_header_.last_packet = packet.pc11.ucLastPacket;

    // Misspellings are in original header
    // Byte length is encoded in top 3 bits
    pointcloud_header_.bytes_per_target =
        RAF_COM_CALC_GetBytesLengthOfTraget(packet.pc11.ucTrgtFmtFrmType);

    // I can't find the documentation for this field, but this is
    // how it's done in radar_api.py
    pointcloud_header_.sequence_type.data =
        (packet.pc11.ucTrgtFmtFrmType & 0x1f);

    pointcloud_header_.crd_count = packet.pc11.usCrdCount;

    // Derived parameter
    pointcloud_header_.target_count = static_cast<uint32_t>(
        (packet.pc11.unLength - sizeof(TPointCloud_V1_1)) /
        RAF_COM_CALC_GetBytesLengthOfTraget(packet.pc11.ucTrgtFmtFrmType));

    // Publish header metadata
    // This topic can be useful for diagnostics.
    pointcloud_header_publisher_->publish(pointcloud_header_);
  }

  void clear_pointcloud() {
    current_pointcloud_sph_.clear();
    current_pointcloud_cart_.clear();
  }

  // Convert from binary packet to spherical coordinates and
  // accumulate points.
  void add_sph_points_from_packet(RafPacket &packet) {
    // Arbe's function returns a a pointer to an array of this layout, it's
    // a bit unusual.
    struct {
      // The exact layout of this struct is important
      TargetGenericResolution resolution;
      TargetGeneric targets[MAX_PACKET_TARGETS];
    } resolution_targets;

    // Function returns size in bytes to we need to convert that
    // to count further down.
    uint32_t targets_size = 0;
    uint32_t targets_count = 0;

    // This function is quite strange. Look at the source.
    RAF_COM_CALC_Binary2Phy(reinterpret_cast<uint8_t *>(&packet),
                            reinterpret_cast<uint8_t *>(&resolution_targets),
                            &targets_size);

    if (targets_size == 0) {
      RCLCPP_INFO(get_logger(), "No targets in packet");
      return;
    }

    // Calculate number of targets
    targets_count = static_cast<uint32_t>(
        (static_cast<size_t>(targets_size) - sizeof(TargetGenericResolution)) /
        sizeof(TargetGeneric));

    // Append to point cloud accumulator
    std::copy(resolution_targets.targets,
              resolution_targets.targets + targets_count,
              std::back_inserter(current_pointcloud_sph_));
  }

  // Convert from binary packet to Cartesian coordinates and
  // accumulate points.
  void add_cart_points_from_packet(RafPacket &packet) {
    // Container for cartesian targets.
    TargetCartesian targets[MAX_PACKET_TARGETS];
    // Size in bytes of output
    uint32_t targets_size = 0;
    uint32_t targets_count = 0;

    RAF_COM_CALC_Binary2Cart(reinterpret_cast<uint8_t *>(&packet),
                             reinterpret_cast<uint8_t *>(&targets),
                             &targets_size);

    if (targets_size == 0) {
      RCLCPP_INFO(get_logger(), "No targets in packet");
      return;
    }

    // Cartesian count
    targets_count = static_cast<uint32_t>(static_cast<size_t>(targets_size) /
                                          sizeof(TargetCartesian));

    // Append Cartesian targets to accumulator.
    std::copy(targets, targets + targets_count,
              std::back_inserter(current_pointcloud_cart_));
  }

  void add_points_from_packet(RafPacket &packet) {
    // Update last seen timestamp
    last_seen_timestamp_ = packet.timestamp();
    // Parse points to point accumulator vectors
    add_sph_points_from_packet(packet);
    add_cart_points_from_packet(packet);
  }

  // State machine for packet handling
  void handle_event(Event::V event) {
    // Visitor for current state. This pattern is quite neat as it
    // checks that all states and events are handled at compile
    // time.
    std::visit(
        overloaded{
            [&](State::Idle &idle) {
              (void)idle;
              // Check event
              std::visit(
                  overloaded{
                      [&](Event::PointcloudPacket &packet_event) {
                        RCLCPP_INFO(get_logger(), "New point cloud frame: %d",
                                    packet_event.packet.frame_counter());

                        // In idle state, expect message with number 0.
                        if (packet_event.packet.message_number() == 0 and
                            packet_event.packet.is_last_packet()) {
                          // Single packet pointcloud.
                          // Add points and publish
                          add_points_from_packet(packet_event.packet);

                          RCLCPP_INFO(get_logger(),
                                      "Publishing single packet pointcloud "
                                      "with %ld points.",
                                      current_pointcloud_sph_.size());

                          publish_pointcloud();
                          // Stay in idle state.
                        } else if (packet_event.packet.message_number() == 0) {
                          // First packet of multi packet pointcloud.
                          add_points_from_packet(packet_event.packet);
                          // Change state
                          current_state_ = State::In_Pointcloud{
                              packet_event.packet.frame_counter(),
                              packet_event.packet.message_number(),
                          };
                        } else {
                          // Received packet with non-zero
                          // message number in idle (expected 0)
                          RCLCPP_WARN(get_logger(),
                                      "Received packet with message number %d "
                                      "in idle state (expected 0).",
                                      packet_event.packet.message_number());

                          // Add points anyway
                          add_points_from_packet(packet_event.packet);
                          // Change state
                          current_state_ = State::In_Pointcloud{
                              packet_event.packet.frame_counter(),
                              packet_event.packet.message_number(),
                          };
                        }
                      },
                      [](Event::Timeout &timeout) {
                        (void)timeout;
                        // TODO.
                      },
                  },
                  event);
            },
            [&](State::In_Pointcloud &in_pointcloud) {
              // Check event
              std::visit(
                  overloaded{
                      [&](Event::PointcloudPacket &packet_event) {
                        // Print warning if number of packets larger than
                        // allocated QOS_BACKLOG
                        if (in_pointcloud.message_number >
                            QOS_BACKLOG_INCOMING) {
                          RCLCPP_WARN(
                              get_logger(),
                              "Number of UDP packets larger than buffer size.");
                        }

                        // Check that we are in the same frame
                        if (in_pointcloud.frame_counter !=
                            packet_event.packet.frame_counter()) {
                          RCLCPP_WARN(get_logger(),
                                      "Unexpected frame_counter, publishing "
                                      "incomplete pointcloud");

                          // Previously this was clear_pointcloud();
                          // Publish what we have, might be incomplete.
                          publish_pointcloud();

                          // Wait for next frame.
                          current_state_ = State::Idle{};

                        } else if ((in_pointcloud.message_number + 1) !=
                                       packet_event.packet.message_number() &&
                                   packet_event.packet.is_last_packet()) {

                          RCLCPP_WARN(
                              get_logger(),
                              "Unexpected message_number %d in frame %d.",
                              packet_event.packet.message_number(),
                              packet_event.packet.frame_counter());

                          add_points_from_packet(packet_event.packet);

                          publish_pointcloud();

                          current_state_ = State::Idle{};

                        } else if ((in_pointcloud.message_number + 1) !=
                                   packet_event.packet.message_number()) {
                          RCLCPP_WARN(
                              get_logger(),
                              "Unexpected message_number %d in frame %d.",
                              packet_event.packet.message_number(),
                              packet_event.packet.frame_counter());

                          add_points_from_packet(packet_event.packet);

                          // Modify current state
                          in_pointcloud.message_number =
                              packet_event.packet.message_number();

                        } else if (packet_event.packet.is_last_packet()) {
                          // Last packet
                          add_points_from_packet(packet_event.packet);

                          RCLCPP_INFO(get_logger(),
                                      "Publishing multi packet pointcloud with "
                                      "%ld points.",
                                      current_pointcloud_sph_.size());

                          publish_pointcloud();
                          // Change state.
                          current_state_ = State::Idle{};

                        } else {
                          // Packet as expected
                          add_points_from_packet(packet_event.packet);
                          // Modify current state
                          in_pointcloud.message_number =
                              packet_event.packet.message_number();
                        }
                      },
                      [](Event::Timeout &timeout) {
                        (void)timeout;
                        // TODO.
                      },
                  },
                  event);
            },
            [&](State::Error &error) {
              (void)error;
              // Check event
              std::visit(overloaded{
                             [&](Event::PointcloudPacket &packet_event) {
                               (void)packet_event;
                               RCLCPP_ERROR(get_logger(),
                                            "Packet event in error state.");

                               // Probably jump to idle state.
                             },
                             [&](Event::Timeout &timeout) {
                               (void)timeout;
                               RCLCPP_ERROR(get_logger(),
                                            "Timeout event in error state");
                             },
                         },
                         event);
            }},
        current_state_);
  }

  // TODO: Change to use data.
  RafPacket parse_raf_packet(const std::vector<uint8_t> &data) const {
    RafPacket raf_packet;

    // The "serialization" relies on exact memory layout.
    std::memcpy(&raf_packet, data.data(), data.size());

    // Verify packet prefix is as expected.
    if (raf_packet.pc11.usPrefix != 0xa55a) {
      RCLCPP_ERROR(get_logger(), "Skipping packet with invalid prefix: %d",
                   raf_packet.pc11.usPrefix);
      throw std::runtime_error("Invalid packet prefix");
    }

    // Verify that packet length is expected.
    if (raf_packet.pc11.unLength != data.size()) {
      RCLCPP_ERROR(get_logger(), "Skipping packet with invalid length: %d",
                   raf_packet.pc11.unLength);
      throw std::runtime_error("Invalid packet length");
    }

    // TODO: verify CRC checksum when Arbe has fixed this.

    return raf_packet;
  }

  void udp_packet_callback(
      const raf_interfaces::msg::UDPPacket::SharedPtr udp_packet) {

    // This seems to be a fixed size limit of the radar, but add
    // this exception to make sure.
    if (udp_packet->data.size() > MAX_PACKET_SIZE) {
      throw std::runtime_error("Packet size exceeds maximum");
    }

    auto packet = parse_raf_packet(udp_packet->data);

    // Check packet type.
    switch (packet.pc11.usType) {
    case PointCloud_Output_V1_1: {
      if (publish_header_) {
        publish_header_from_packet(packet);
      }
      // Push event to state machine.
      auto pointcloud_packet_event = Event::PointcloudPacket{packet};
      handle_event(pointcloud_packet_event);
      break;
    }
    case AlgoDetectorThr_Output: {
      RCLCPP_INFO(get_logger(), "Skipping AlgoDetectorThr_Output packet");
      break;
    }
    }
  }

public:
  RafPointcloudParserNode(bool publish_pointcloud = true,
                          bool publish_header = true)
      : Node("raf_pointcloud_parser_node"), pointcloud_modifier_(pointcloud_) {
    RCLCPP_INFO(get_logger(), "raf_pointcloud_parser_node has started");

    auto frame_id = declare_parameter("frame_id", "radar");

    // Publish options
    publish_pointcloud_ =
        declare_parameter("publish_pointcloud", publish_pointcloud);
    publish_header_ = declare_parameter("publish_header", publish_header);

    // Prepare pointcloud2 message
    // Reserve a somehwat arbitrary amount of points
    // TODO: Settle on a good value
    current_pointcloud_sph_.reserve(MAX_POINTCLOUD_SIZE);
    current_pointcloud_cart_.reserve(MAX_POINTCLOUD_SIZE);

    // Reserve memory for pointcloud2
    pointcloud_modifier_.reserve(MAX_POINTCLOUD_SIZE);
    pointcloud_.header.frame_id = frame_id;
    // Prepare meta data message with same frame_id
    pointcloud_header_.header.frame_id = frame_id;

    // Define pointcloud2 layout
    using PointField = sensor_msgs::msg::PointField;
    pointcloud_modifier_.setPointCloud2Fields(
        8, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1,
        PointField::FLOAT32, "range", 1, PointField::FLOAT32, "elevation", 1,
        PointField::FLOAT32, "azimuth", 1, PointField::FLOAT32, "power", 1,
        PointField::FLOAT32, "doppler", 1, PointField::FLOAT32);

    // Create pointcloud publisher
    pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data", QOS_BACKLOG_OUTGOING);

    udp_packet_subscription_ =
        create_subscription<raf_interfaces::msg::UDPPacket>(
            "udp_packet", QOS_BACKLOG_INCOMING,
            std::bind(&RafPointcloudParserNode::udp_packet_callback, this,
                      std::placeholders::_1));

    // Create pointcloud meta data publisher
    pointcloud_header_publisher_ =
        create_publisher<raf_interfaces::msg::TPointCloudV11>(
            "radar_header", QOS_BACKLOG_OUTGOING);
  }

  // Virtual destructor
  virtual ~RafPointcloudParserNode() {}
};
