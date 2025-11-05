// Copyright (c) Sensrad 2023
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <glibmm.h>

// Standard messagse
#include <std_msgs/msg/header.hpp>
// Sensor messages
#include <sensor_msgs/msg/point_cloud2.hpp>
// Hugin messages
#include <raf_interfaces/msg/control_state.hpp>
#include <raf_interfaces/msg/t_point_cloud_v11.hpp>

// Hugin interfaces
#include <raf_interfaces/srv/rdr_ctrl_set_active_seq.hpp>
#include <raf_interfaces/srv/rdr_ctrl_set_thresholds.hpp>
#include <raf_interfaces/srv/rdr_ctrl_start_tx.hpp>
#include <raf_interfaces/srv/rdr_ctrl_stop_tx.hpp>
#include <raf_interfaces/srv/sys_cfg_set_time.hpp>

#include <chrono>
#include <future>
#include <mutex>

#include <boost/format.hpp>

using namespace std::literals::chrono_literals;

class RAFGuiNode : public rclcpp::Node {
public:
  // Copied from Arbe's code to avoid circular dependency
  enum ESeuqenceType {
    IdleSeq = 0,
    CoarseShortSeq,
    CoarseMidSeq,
    CoarseLongSeq,
    FineShortSeq,
    FineMidSeq,
    FineLongSeq,
    CalibrationSeq,
    LabSeq,
    DelayCalibrationSeq,
    CoarseUltraLongSeq,
    FineUltraLongSeq,
    UserConfigureSeq1,
    UserConfigureSeq2,
    CalibrationFastSeq,
    LastSeq
  };

  // Shorthand for signals
  using PointCloud2SharedPtr = sensor_msgs::msg::PointCloud2::SharedPtr;
  using ControlStateSharedPtr = raf_interfaces::msg::ControlState::SharedPtr;

  // Signal types
  using signal_on_pointcloud_t =
      sigc::signal<void(PointCloud2SharedPtr pointcloud2)>;
  using signal_on_control_state_t =
      sigc::signal<void(ControlStateSharedPtr control_state)>;
  using signal_on_t11_pointcloud_t = sigc::signal<void>;
  using signal_on_message_t = sigc::signal<void, std::string>;

private:
  constexpr static int QOS_BACKLOG = 10; // QoS backlog size

  // Signals (accessors further down)
  signal_on_pointcloud_t signal_on_pointcloud_;
  signal_on_control_state_t signal_on_control_state_;
  signal_on_t11_pointcloud_t signal_on_t11_pointcloud_;
  signal_on_message_t signal_on_message_;

  // Node lock
  std::mutex node_mutex_;

  rclcpp::TimerBase::SharedPtr alive_timer_;

  // ROS2 clients
  rclcpp::Client<raf_interfaces::srv::RdrCtrlStartTx>::SharedPtr
      start_tx_client_;
  rclcpp::Client<raf_interfaces::srv::RdrCtrlStopTx>::SharedPtr stop_tx_client_;
  rclcpp::Client<raf_interfaces::srv::RdrCtrlSetActiveSeq>::SharedPtr
      set_active_seq_client_;
  rclcpp::Client<raf_interfaces::srv::RdrCtrlSetThresholds>::SharedPtr
      set_thresholds_client_;
  rclcpp::Client<raf_interfaces::srv::SysCfgSetTime>::SharedPtr
      set_time_client_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_subscription_;
  rclcpp::Subscription<raf_interfaces::msg::TPointCloudV11>::SharedPtr
      header_subscription_;
  rclcpp::Subscription<raf_interfaces::msg::ControlState>::SharedPtr
      control_state_subscription_;

  // Timer for rate limiting requests.
  rclcpp::TimerBase::SharedPtr rate_timer_;

  // Emit a log message to be shown in the GUI.
  void emit_message(std::string &message) {
    // Emit message signal on main thread
    // Create a message that will outlive the callback
    std::shared_ptr<std::string> shared_msg =
        std::make_shared<std::string>(message);
    Glib::MainContext::get_default()->invoke([this, shared_msg]() {
      // Signal emitted on glib main thread
      signal_on_message_.emit(*shared_msg);
      return false; // false = single shot
    });
  }

  // Prune expired requests
  void on_alive_timer() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // Clear expired requests
    const auto expiration_time = std::chrono::system_clock::now() - 2s;
    std::size_t num_expired = 0;

    RCLCPP_DEBUG(get_logger(), "Pruning expired requests...");

    std::vector<int64_t> pruned_requests;
    // There's a bug in the prune_requests_older_than function.
    // https://github.com/ros2/rclcpp/issues/2007
    num_expired += start_tx_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);
    num_expired += stop_tx_client_->prune_requests_older_than(expiration_time,
                                                              &pruned_requests);
    num_expired += set_active_seq_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);
    num_expired += set_thresholds_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);
    num_expired += set_time_client_->prune_requests_older_than(
        expiration_time, &pruned_requests);

    RCLCPP_DEBUG(get_logger(), "Expired %ld requests", num_expired);

    // Emit message for gui
    if (num_expired > 0) {
      std::string message =
          (boost::format("Connection to control node timed out (%d requests)") %
           num_expired)
              .str();
      emit_message(message);
    }
  }

  // Subscription callbacks
  void on_header(const raf_interfaces::msg::TPointCloudV11::SharedPtr msg) {
    Glib::MainContext::get_default()->invoke([this, msg]() {
      signal_on_t11_pointcloud().emit();
      return false;
    });
  }

  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Glib::MainContext::get_default()->invoke([this, msg]() {
      // The ref count on msg is increased and will be valid until emit return.
      signal_on_pointcloud().emit(msg);
      return false;
    });
  }

  void
  on_control_state(const raf_interfaces::msg::ControlState::SharedPtr msg) {
    // The ref count on msg is increased and will be valid until emit return.
    Glib::MainContext::get_default()->invoke([this, msg]() {
      signal_on_control_state().emit(msg);

      return false;
    });
  }

  // Client slots
  void invoke_rate_limit(std::function<void()> &&callback) {
    rate_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        // capture shared_from_this() to keep the node alive
        // capture this for ease of access to the timer
        // move callback into lambda to avoid copying
        [lifetime = shared_from_this(), this, cb = std::move(callback)]() {
          cb();
          rate_timer_->cancel();
        });
  }

public:
  // Signal accessors
  signal_on_pointcloud_t &signal_on_pointcloud() {
    return signal_on_pointcloud_;
  }

  signal_on_control_state_t &signal_on_control_state() {
    return signal_on_control_state_;
  }

  signal_on_t11_pointcloud_t &signal_on_t11_pointcloud() {
    return signal_on_t11_pointcloud_;
  }

  signal_on_message_t &signal_on_message() { return signal_on_message_; }

  RAFGuiNode(const std::string &node_name) : Node(node_name) {

    RCLCPP_INFO(get_logger(), "RAF GUI node started");

    std::lock_guard<std::mutex> lock(node_mutex_);

    // Create subscriptions
    header_subscription_ =
        create_subscription<raf_interfaces::msg::TPointCloudV11>(
            "radar_header", QOS_BACKLOG,
            std::bind(&RAFGuiNode::on_header, this, std::placeholders::_1));

    pointcloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "radar_data", QOS_BACKLOG,
            std::bind(&RAFGuiNode::on_pointcloud, this, std::placeholders::_1));

    // Control state has transient local QoS.
    auto control_state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    control_state_qos.transient_local();

    control_state_subscription_ =
        create_subscription<raf_interfaces::msg::ControlState>(
            "control_state", control_state_qos,
            std::bind(&RAFGuiNode::on_control_state, this,
                      std::placeholders::_1));

    // Create a timer that will fire every 100ms.
    alive_timer_ =
        create_wall_timer(1000ms, std::bind(&RAFGuiNode::on_alive_timer, this));
    // Clients
    // Start tx client
    start_tx_client_ =
        create_client<raf_interfaces::srv::RdrCtrlStartTx>("start");
    // Stop tx client
    stop_tx_client_ = create_client<raf_interfaces::srv::RdrCtrlStopTx>("stop");
    // Set active seq client
    set_active_seq_client_ =
        create_client<raf_interfaces::srv::RdrCtrlSetActiveSeq>(
            "set_active_seq");
    // Set thresholds client
    set_thresholds_client_ =
        create_client<raf_interfaces::srv::RdrCtrlSetThresholds>(
            "set_thresholds");
    // Set time client
    set_time_client_ =
        create_client<raf_interfaces::srv::SysCfgSetTime>("set_time");

    RCLCPP_INFO(get_logger(), "RAF GUI node initialized");
  }

  virtual ~RAFGuiNode() {}

  // Slots
  void start_tx() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // This will be executed on the ros2 run loop
    invoke_rate_limit([this]() {
      const auto request =
          std::make_shared<raf_interfaces::srv::RdrCtrlStartTx::Request>();
      start_tx_client_->async_send_request(request);
    });
  }

  void stop_tx() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    invoke_rate_limit([this]() {
      const auto request =
          std::make_shared<raf_interfaces::srv::RdrCtrlStopTx::Request>();
      stop_tx_client_->async_send_request(request);
    });
  }

  void set_time() {
    std::lock_guard<std::mutex> lock(node_mutex_);

    invoke_rate_limit([this]() {
      const auto request =
          std::make_shared<raf_interfaces::srv::SysCfgSetTime::Request>();
      request->use_ros2_time = true;
      set_time_client_->async_send_request(request);
    });
  }

  void set_sequence(ESeuqenceType sequence_type) {
    std::lock_guard<std::mutex> lock(node_mutex_);

    invoke_rate_limit([this, sequence_type]() {
      const auto request =
          std::make_shared<raf_interfaces::srv::RdrCtrlSetActiveSeq::Request>();
      request->sequence_type = sequence_type;
      set_active_seq_client_->async_send_request(request);
    });
  }

  void set_threshold(const float static_threshold, const float dynamic_azimuth,
                     const float dynamic_elevation) {
    std::lock_guard<std::mutex> lock(node_mutex_);

    // This will be executed on the ros2 run loop
    invoke_rate_limit(
        [this, static_threshold, dynamic_azimuth, dynamic_elevation]() {
          const auto request = std::make_shared<
              raf_interfaces::srv::RdrCtrlSetThresholds::Request>();

          request->static_threshold = static_threshold;
          request->dynamic_azimuth = dynamic_azimuth;
          request->dynamic_elevation = dynamic_elevation;

          set_thresholds_client_->async_send_request(request);
        });
  }
};
