// Copyright (c) Sensrad 2023-2024

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <raf_interfaces/srv/rdr_ctrl_set_active_seq.hpp>
#include <raf_interfaces/srv/rdr_ctrl_set_rsl.hpp>
#include <raf_interfaces/srv/rdr_ctrl_set_thresholds.hpp>
#include <raf_interfaces/srv/rdr_ctrl_start_tx.hpp>
#include <raf_interfaces/srv/rdr_ctrl_stop_tx.hpp>
#include <raf_interfaces/srv/sys_cfg_local_max.hpp>
#include <raf_interfaces/srv/sys_cfg_set_adt.hpp>
#include <raf_interfaces/srv/sys_cfg_set_time.hpp>

#include <raf_interfaces/msg/control_state.hpp>
#include <raf_interfaces/msg/sequence_type.hpp>

#include <memory.h>
#include <optional>
#include <unordered_map>
#include <unordered_set>

#include <boost/config.hpp>

#include "radar_client.hpp"

class RafControlNode : public rclcpp::Node {

  // Optional bool type, used for settings that might not be available
  // on all versions.
  typedef std::optional<bool> bool_opt_t;
  typedef struct fw_settings_t {
    // Existing fields
    bool ntc_3d;
    bool ntc_4d;
    bool cfar_3d;
    bool cfar_4d;
    float bias_4d;
    uint8_t ntc_percentage;

    // New fields for firmware 1.8.1
    bool_opt_t adt;

    // New fields for Firmware 1.8.5
    bool_opt_t local_max;
    std::optional<uint32_t> az_cdf_frame_type;
    std::optional<std::array<uint16_t, AZ_CDF_NUM_OF_THR_LEVELS>>
        az_cdf_thr_levels;
    std::optional<uint32_t> range_hist_start_offset;
    std::optional<uint32_t> range_hist_increment_level;
    bool_opt_t enable_az_hist;
    bool_opt_t enable_range_hist;
    bool_opt_t enable_spot;
    bool_opt_t enable_rsl;
    std::optional<uint32_t> rsl_back_off;

    // Constructor to initialize default values
    fw_settings_t() = default;

  } fw_settings_t;

  fw_settings_t buildFwSettings_1_7_1() {
    fw_settings_t settings{};
    settings.ntc_3d = false;
    settings.ntc_4d = true;
    settings.cfar_3d = true;
    settings.cfar_4d = true;
    settings.bias_4d = 4;
    settings.ntc_percentage = 2;
    return settings;
  }

  fw_settings_t buildFwSettings_1_8_1() {
    fw_settings_t settings{};
    settings.ntc_3d = false;
    settings.ntc_4d = true;
    settings.cfar_3d = true;
    settings.cfar_4d = true;
    settings.bias_4d = 0;
    settings.ntc_percentage = 2;
    settings.adt = false;
    return settings;
  }

  fw_settings_t buildFwSettings_1_8_5() {
    fw_settings_t settings{};
    settings.ntc_3d = false;
    settings.ntc_4d = true;
    settings.cfar_3d = true;
    settings.cfar_4d = true;
    settings.bias_4d = 0;
    settings.ntc_percentage = 2;
    settings.adt = false;
    settings.enable_az_hist = false;
    settings.enable_range_hist = false;
    settings.enable_spot = true;
    settings.enable_rsl = true;
    return settings;
  }

  // Map of firmware versions to settings
  const std::unordered_map<std::string, fw_settings_t> fw_settings_map_{
      {"1.7.1", buildFwSettings_1_7_1()},
      {"1.8.1", buildFwSettings_1_8_1()},
      {"1.8.5", buildFwSettings_1_8_5()}};

  // Active firmware version.
  std::string fw_version_;
  fw_settings_t fw_settings_;

  // Parameters
  // Connect to radar on this IP address
  std::string ip_;
  // On this port
  std::uint16_t port_;

  // Client
  std::shared_ptr<hugin::RadarClient> radar_client_;

  // User settable sequence types
  const std::unordered_set<ESeuqenceType> allowed_sequence_types_{
      IdleSeq,       CoarseShortSeq,     CoarseMidSeq,
      CoarseLongSeq, FineShortSeq,       FineMidSeq,
      FineLongSeq,   CoarseUltraLongSeq, FineUltraLongSeq};

  // Default start parameters
  ESeuqenceType default_active_seq_;
  bool default_start_tx_;

  // Control state
  rclcpp::Publisher<raf_interfaces::msg::ControlState>::SharedPtr
      control_state_pub_;
  raf_interfaces::msg::ControlState control_state_msg_;

  // Radar control services
  rclcpp::Service<raf_interfaces::srv::RdrCtrlStartTx>::SharedPtr
      start_service_;
  rclcpp::Service<raf_interfaces::srv::RdrCtrlStopTx>::SharedPtr stop_service_;
  rclcpp::Service<raf_interfaces::srv::RdrCtrlSetActiveSeq>::SharedPtr
      set_active_seq_service_;
  rclcpp::Service<raf_interfaces::srv::RdrCtrlSetThresholds>::SharedPtr
      set_thresholds_service_;
  // Sys config services
  rclcpp::Service<raf_interfaces::srv::SysCfgSetTime>::SharedPtr
      set_time_service_;
  // From firmware version 1.8.5 services
  rclcpp::Service<raf_interfaces::srv::SysCfgSetAdt>::SharedPtr
      set_adt_service_;
  rclcpp::Service<raf_interfaces::srv::SysCfgLocalMax>::SharedPtr
      set_local_max_service_;
  rclcpp::Service<raf_interfaces::srv::RdrCtrlSetRsl>::SharedPtr
      set_rsl_service_;

  // Return ROS2 time in ms
  // We need this as a static function to fit in the Arbe API.
  static uint64_t get_ros2_time_ms() {
    // Nanoseconds since Unix Epoch on January 1st, 1970 at UTC.
    uint64_t now_ns =
        static_cast<uint64_t>(rclcpp::Clock().now().nanoseconds());
    // Scale to milliseconds, there's a precision loss here.
    return now_ns / 1'000'000;
  }

  // Publish best known control state message
  void publish_control_state() {
    control_state_msg_.header.stamp = rclcpp::Clock().now();
    control_state_pub_->publish(control_state_msg_);
  }

  // TODO: move to separate file
  void create_services() {
    // Services with lambda callbacks
    start_service_ = create_service<raf_interfaces::srv::RdrCtrlStartTx>(
        "start",
        [&](const std::shared_ptr<raf_interfaces::srv::RdrCtrlStartTx::Request>,
            std::shared_ptr<raf_interfaces::srv::RdrCtrlStartTx::Response>
                response) {
          try {
            radar_client_->radar_start_tx();

            control_state_msg_.tx_enabled = true;

            response->success = true;
            response->message = "start";
          } catch (const hugin::Ack1Timeout &) {
            RCLCPP_WARN(get_logger(),
                        "start_tx ack1 timeout, radar already started?");
            response->success = false;
            response->message = "start_tx ack1 timeout, radar already started?";
          } catch (const hugin::Timeout &) {
            RCLCPP_WARN(get_logger(), "start_tx timeout");
            response->success = false;
            response->message = "start_tx timeout";
          }

          publish_control_state();
        });

    stop_service_ = create_service<raf_interfaces::srv::RdrCtrlStopTx>(
        "stop",
        [&](const std::shared_ptr<raf_interfaces::srv::RdrCtrlStopTx::Request>,
            std::shared_ptr<raf_interfaces::srv::RdrCtrlStopTx::Response>
                response) {
          try {
            radar_client_->radar_stop_tx();

            control_state_msg_.tx_enabled = false;

            response->success = true;
            response->message = "stopped";
          } catch (const hugin::Timeout &) {
            RCLCPP_WARN(get_logger(), "stop_tx request timed out");
            response->success = false;
            response->message = "stop_tx request timed out";
          }

          publish_control_state();
        });

    set_active_seq_service_ = create_service<
        raf_interfaces::srv::RdrCtrlSetActiveSeq>(
        "set_active_seq",
        [&](const std::shared_ptr<
                raf_interfaces::srv::RdrCtrlSetActiveSeq::Request>
                request,
            std::shared_ptr<raf_interfaces::srv::RdrCtrlSetActiveSeq::Response>
                response) {
          const auto sequence_type =
              static_cast<ESeuqenceType>(request->sequence_type);
          // Check if sequence type is allowed
          if (allowed_sequence_types_.find(sequence_type) !=
              allowed_sequence_types_.end()) {
            try {
              radar_client_->radar_set_active_seq(
                  static_cast<ESeuqenceType>(request->sequence_type));

              control_state_msg_.active_seq.data = sequence_type;

              response->success = true;
              response->message = "set_active";
            } catch (const hugin::Timeout &) {
              RCLCPP_WARN(get_logger(), "set_active_seq request timed out");
              response->success = false;
              response->message = "set_active_seq request timed out";
            }
          } else {
            RCLCPP_WARN(get_logger(),
                        "set_active_seq: sequence type not allowed");
            response->success = false;
            response->message = "set_active_seq: sequence type not allowed";
          }

          publish_control_state();
        });

    set_thresholds_service_ = create_service<
        raf_interfaces::srv::RdrCtrlSetThresholds>(
        "set_thresholds",
        [&](const std::shared_ptr<
                raf_interfaces::srv::RdrCtrlSetThresholds::Request>
                request,
            std::shared_ptr<raf_interfaces::srv::RdrCtrlSetThresholds::Response>
                response) {
          try {
            radar_client_->radar_set_thresholds(
                fw_settings_.bias_4d, request->static_threshold,
                request->dynamic_azimuth, request->dynamic_elevation);

            control_state_msg_.thresholds.static_threshold =
                request->static_threshold;
            control_state_msg_.thresholds.dynamic_azimuth =
                request->dynamic_azimuth;
            control_state_msg_.thresholds.dynamic_elevation =
                request->dynamic_elevation;

            response->success = true;
            response->message = "set_thresholds";
          } catch (const hugin::Timeout &) {
            RCLCPP_WARN(get_logger(), "set_thresholds request timed out");
            response->success = false;
            response->message = "set_thresholds request timed out";
          }

          publish_control_state();
        });

    set_time_service_ = create_service<raf_interfaces::srv::SysCfgSetTime>(
        "set_time",
        [&](const std::shared_ptr<raf_interfaces::srv::SysCfgSetTime::Request>
                request,
            std::shared_ptr<raf_interfaces::srv::SysCfgSetTime::Response>
                response) {
          const auto time_ms =
              request->use_ros2_time ? get_ros2_time_ms() : request->time_ms;

          try {
            radar_client_->radar_sync_time_ms(time_ms);

            control_state_msg_.timestamp_ms = time_ms;

            response->success = true;
            response->message = "set time " + std::to_string(time_ms) + "ms";
          } catch (const hugin::Timeout &) {
            RCLCPP_WARN(get_logger(), "set_time request timed out");
            response->success = false;
            response->message = "set_time request timed out";
          }

          publish_control_state();
        });

    set_adt_service_ = create_service<raf_interfaces::srv::SysCfgSetAdt>(
        "set_adt",
        [&](const std::shared_ptr<raf_interfaces::srv::SysCfgSetAdt::Request>
                request,
            std::shared_ptr<raf_interfaces::srv::SysCfgSetAdt::Response>
                response) {
          try {
            radar_client_->radar_set_adt(request->enable);

            control_state_msg_.adt_enabled = request->enable;

            response->success = true;
            response->message = "set adt " + std::to_string(request->enable);
          } catch (const hugin::Timeout &) {
            RCLCPP_WARN(get_logger(), "set_adt request timed out");
            response->success = false;
            response->message = "set_adt request timed out";
          }

          publish_control_state();
        });

    set_rsl_service_ = create_service<raf_interfaces::srv::RdrCtrlSetRsl>(
        "set_rsl",
        [&](const std::shared_ptr<raf_interfaces::srv::RdrCtrlSetRsl::Request>
                request,
            std::shared_ptr<raf_interfaces::srv::RdrCtrlSetRsl::Response>
                response) {
          try {
            radar_client_->radar_set_rsl_params(request->rsl_back_off);
            control_state_msg_.spot_rsl.rsl_back_off = request->rsl_back_off;

            response->success = true;
            response->message = "set rsl";
          } catch (const hugin::Timeout &) {
            RCLCPP_WARN(get_logger(), "set_rsl request timed out");
            response->success = false;
            response->message = "set_rsl request timed out";
          }

          publish_control_state();
        });
  }

  void startup_commands() {
    // Set startup commands. The commands are queued in this
    // order and sent on successful connect.

    const auto time_ms = get_ros2_time_ms();

    const float default_static_threshold = 5.0;
    const float default_dynamic_azimuth = 20.0;
    const float default_dynamic_elevation = 5.0;

    radar_client_->radar_stop_tx();
    control_state_msg_.tx_enabled = false;

    radar_client_->radar_sync_time_ms(time_ms);
    control_state_msg_.timestamp_ms = time_ms;

    // Should this be part of settings map?
    radar_client_->radar_set_packet_format(false, 2);
    // Not include in control state message

    radar_client_->radar_set_active_seq(default_active_seq_);
    control_state_msg_.active_seq.data = default_active_seq_;

    // Maybe store default thresholds in settings map?
    radar_client_->radar_set_thresholds(
        fw_settings_.bias_4d, default_static_threshold, default_dynamic_azimuth,
        default_dynamic_elevation);

    control_state_msg_.thresholds.static_threshold = default_static_threshold;
    control_state_msg_.thresholds.dynamic_azimuth = default_dynamic_azimuth;
    control_state_msg_.thresholds.dynamic_elevation = default_dynamic_elevation;

    radar_client_->radar_set_ntc_mode(fw_settings_.ntc_3d, fw_settings_.ntc_4d,
                                      fw_settings_.ntc_percentage, false);
    control_state_msg_.ntc.ntc_3d_enabled = fw_settings_.ntc_3d;
    control_state_msg_.ntc.ntc_4d_enabled = fw_settings_.ntc_4d;
    control_state_msg_.ntc.ntc_percentage = fw_settings_.ntc_percentage;
    control_state_msg_.ntc.send_metadata = false;

    radar_client_->radar_set_cfar_mode(fw_settings_.cfar_3d,
                                       fw_settings_.cfar_4d, false);
    control_state_msg_.cfar.cfar_3d_enabled = fw_settings_.cfar_3d;
    control_state_msg_.cfar.cfar_4d_enabled = fw_settings_.cfar_4d;
    control_state_msg_.cfar.send_metadata = false;

    // Check if ADT is available
    if (fw_settings_.adt) {
      try {
        radar_client_->radar_set_adt(fw_settings_.adt.value());
        control_state_msg_.adt_enabled = fw_settings_.adt.value();
      } catch (const hugin::Ack1Timeout &) {
        RCLCPP_WARN(get_logger(),
                    "radar_set_adt ack1 timeout, ADT not available?");
      }
    }

    // Check if local max is available
    if (fw_settings_.local_max) {
      try {
        radar_client_->radar_set_local_max(fw_settings_.local_max.value());
        control_state_msg_.local_max_enabled = fw_settings_.local_max.value();
      } catch (const hugin::Ack1Timeout &) {
        RCLCPP_WARN(
            get_logger(),
            "radar_set_local_max ack1 timeout, LOCAL MAX not available?");
      }
    }
    // New settings for Firmware 1.8.5, conditionally applied
    if (fw_settings_.az_cdf_frame_type && fw_settings_.az_cdf_thr_levels) {
      radar_client_->radar_set_az_cdf_thr_levels(
          fw_settings_.az_cdf_frame_type.value(),
          fw_settings_.az_cdf_thr_levels.value());
    }

    if (fw_settings_.range_hist_start_offset &&
        fw_settings_.range_hist_increment_level) {
      radar_client_->radar_set_range_hist_config(
          fw_settings_.range_hist_start_offset.value(),
          fw_settings_.range_hist_increment_level.value());
    }

    if (fw_settings_.enable_az_hist && fw_settings_.enable_range_hist) {
      radar_client_->radar_set_hist_output(
          fw_settings_.enable_az_hist.value(),
          fw_settings_.enable_range_hist.value());
      control_state_msg_.az_hist_enabled = fw_settings_.enable_az_hist.value();
      control_state_msg_.range_hist_enabled =
          fw_settings_.enable_range_hist.value();
    }

    if (fw_settings_.enable_spot && fw_settings_.enable_rsl) {
      try {
        radar_client_->radar_set_spot_rsl(fw_settings_.enable_spot.value(),
                                          fw_settings_.enable_rsl.value());
        control_state_msg_.spot_rsl.spot_enabled =
            fw_settings_.enable_spot.value();
        control_state_msg_.spot_rsl.rsl_enabled =
            fw_settings_.enable_rsl.value();
      } catch (const hugin::Ack1Timeout &) {
        RCLCPP_WARN(get_logger(),
                    "radar_set_spot_rsl ack1 timeout, SPOT/RSL not available?");
      }
    }

    if (fw_settings_.rsl_back_off) {
      try {
        radar_client_->radar_set_rsl_params(fw_settings_.rsl_back_off.value());
        control_state_msg_.spot_rsl.spot_enabled =
            fw_settings_.enable_spot.value();
        control_state_msg_.spot_rsl.rsl_enabled =
            fw_settings_.enable_rsl.value();
        control_state_msg_.spot_rsl.rsl_back_off =
            fw_settings_.rsl_back_off.value();

      } catch (const hugin::Ack1Timeout &) {
        RCLCPP_WARN(get_logger(),
                    "radar_set_rsl_params ack1 timeout, RSL not available?");
      }
    }

    // Auto-start?
    if (default_start_tx_) {
      radar_client_->radar_start_tx();
      control_state_msg_.tx_enabled = true;
    }

    // Publish control state message
    publish_control_state();
  }

public:
  RafControlNode(const std::string &ip = "10.20.30.40",
                 const std::uint16_t port = 6001,
                 const std::string &fw_version = "1.7.1",
                 const std::string &node_name = "raf_control_node")
      : Node(node_name), fw_version_(fw_version), ip_(ip), port_(port) {
    RCLCPP_INFO(get_logger(), "raf_control_node has been started");

    // Firmware version parameter
    fw_version_ = declare_parameter("fw_version", fw_version);
    // Check that firmware version is supported
    if (const auto settings = fw_settings_map_.find(fw_version_);
        settings != fw_settings_map_.end()) {
      // Set firmware settings
      fw_settings_ = settings->second;
    } else {
      // Unable to continue.
      throw std::runtime_error("Unsupported firmware version: " + fw_version_);
    }

    // Declare parameter create a parameter and also returns
    // the value, either default or what was set.
    ip_ = declare_parameter("ip", ip);
    port_ = static_cast<uint16_t>(declare_parameter("port", port));

    // Allow to auto start tx as a parameter
    default_start_tx_ = declare_parameter("start_tx", false);
    // Allow to set initial active sequence as a parameter
    default_active_seq_ = static_cast<ESeuqenceType>(
        declare_parameter("active_seq", static_cast<int>(FineShortSeq)));

    // Setup publisher with QoS profile durability local
    auto control_state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    control_state_qos.transient_local();
    control_state_pub_ = create_publisher<raf_interfaces::msg::ControlState>(
        "control_state", control_state_qos);

    radar_client_ = std::make_shared<hugin::RadarClient>(get_ros2_time_ms);
    // Connect to radar

    RCLCPP_INFO(get_logger(), "Connecting to radar at %s:%d", ip_.c_str(),
                port_);

    try {
      radar_client_->connect(ip_, port_);
    } catch (const hugin::ConnectTimeout &) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to radar at %s:%d",
                   ip_.c_str(), port_);
      throw; // rethrows
    }

    RCLCPP_INFO(get_logger(), "Connected to radar at %s:%d", ip_.c_str(),
                port_);

    startup_commands();

    // Create ROS2 services
    create_services();
    RCLCPP_INFO(get_logger(), "Services have been created");
  }

  virtual ~RafControlNode() { radar_client_->radar_stop_tx(); }
};
