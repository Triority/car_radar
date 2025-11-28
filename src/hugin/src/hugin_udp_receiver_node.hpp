// Copyright (c) Sensrad 2023
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <mutex>
#include <thread>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <linux/net_tstamp.h>
#include <sys/socket.h>

#include <raf_interfaces/msg/udp_packet.hpp>
#include <std_msgs/msg/header.hpp>

#include "hugin_string_helpers.hpp"

namespace asio = boost::asio;

class UdpReceiverNode : public rclcpp::Node {
  // The maximum size of a UDP packet payload.
  constexpr static size_t MAX_BUFFER_SIZE = 2048;
  constexpr static int QOS_BACKLOG = 300;

  // This is the socket handler run loop.
  asio::io_context io_context_;
  asio::ip::udp::socket socket_;

  std::mutex io_mutex_;
  std::thread io_thread_;

  // These are set through ROS2 parameters.
  bool log_data_;
  bool publish_data_;

  // Packet message
  raf_interfaces::msg::UDPPacket packet_msg_;

  // Publisher of timestamped udp packet
  rclcpp::Publisher<raf_interfaces::msg::UDPPacket>::SharedPtr
      udp_packet_publisher_;

  void handle_read(boost::system::error_code error) {

    if (error == asio::error::operation_aborted) {
      // This is the normal case when the socket is closed.
      RCLCPP_INFO(get_logger(), "UDP socket closed.");
      return;
    } else if (error) {
      // Some other error occurred.
      throw std::runtime_error(error.message());
    }

    // Take the lock, mainly because of the publisher.
    std::scoped_lock lock(io_mutex_);

    // Resize to max packet. Larger packets will be truncated.
    if (packet_msg_.data.size() > 1400) {
      RCLCPP_INFO(get_logger(), "WARNING: packet size larger than expected.");
    }
    packet_msg_.data.resize(MAX_BUFFER_SIZE);

    // Get native socket handle
    int socket_fd = socket_.native_handle();

    // Ancillary data
    char ctrl[MAX_BUFFER_SIZE];
    // Ancillary data structures
    struct msghdr msgh;
    struct iovec iov;
    struct cmsghdr *cmsg;
    struct sockaddr_in src_addr_in;
    ssize_t len;

    // Set up io vector
    iov.iov_base = packet_msg_.data.data();
    iov.iov_len = packet_msg_.data.size();
    // Set up message header
    msgh.msg_iov = &iov;
    msgh.msg_iovlen = 1;
    msgh.msg_control = ctrl;
    msgh.msg_controllen = sizeof(ctrl);
    // From address
    msgh.msg_name = &src_addr_in;
    msgh.msg_namelen = sizeof(src_addr_in);

    // Read packet from socket
    len = recvmsg(socket_fd, &msgh, 0);
    if (len < 0) {
      throw std::runtime_error("Error reading from socket" +
                               std::string(strerror(errno)));
    }

    // Resize message data to actual packet size
    packet_msg_.data.resize(static_cast<size_t>(len));
    // Reset timestamp
    packet_msg_.header.stamp.sec = 0;
    packet_msg_.header.stamp.nanosec = 0;

    // Loop over header data
    for (cmsg = CMSG_FIRSTHDR(&msgh); cmsg != nullptr;
         cmsg = CMSG_NXTHDR(&msgh, cmsg)) {
      // Find timestamp
      if (cmsg->cmsg_level == SOL_SOCKET &&
          cmsg->cmsg_type == SCM_TIMESTAMPNS) {
        struct timespec ts;
        // Get struct timespec from cmsg data
        std::memcpy(&ts, CMSG_DATA(cmsg), sizeof(ts));
        // Set timestamp
        packet_msg_.header.stamp.sec = static_cast<int32_t>(ts.tv_sec);
        packet_msg_.header.stamp.nanosec = static_cast<uint32_t>(ts.tv_nsec);
      }
    }

    // If enabled, log
    if (log_data_) {
      // From address to string (only IPv4)
      char src_addr_str[INET_ADDRSTRLEN];
      // Convert address to string
      inet_ntop(AF_INET, &src_addr_in.sin_addr, src_addr_str, INET_ADDRSTRLEN);
      // Log data
      RCLCPP_INFO(get_logger(), "[%d.%u]: %zu bytes from %s: %s",
                  packet_msg_.header.stamp.sec,
                  packet_msg_.header.stamp.nanosec, (size_t)len, src_addr_str,
                  vector_to_string(packet_msg_.data).c_str());
    }

    // If enabled, publish
    if (publish_data_) {
      // Publish data
      udp_packet_publisher_->publish(packet_msg_);
    }

    // Schedule next read
    read_one_packet();
  }

  void read_one_packet() {
    // Schedule first read. We use async_wait because we need to
    // do a low level read later.
    socket_.async_wait(
        asio::ip::udp::socket::wait_read,
        std::bind(&UdpReceiverNode::handle_read, this, std::placeholders::_1));
  }

public:
  UdpReceiverNode(
      std::string bind = "0.0.0.0", // bind = what interface to bind listen on.
      uint16_t port = 6003)         // what port to listen on
      : Node("udp_receiver_node"), socket_(io_context_) {

    // Declare parameters
    auto bind_ = declare_parameter("bind", bind);
    auto port_ = static_cast<uint16_t>(declare_parameter("port", port));
    auto frame_id = declare_parameter("frame_id", "radar");
    log_data_ = declare_parameter("log_data", true);
    publish_data_ = declare_parameter("publish_data", true);

    asio::ip::udp::endpoint listen_endpoint(
        asio::ip::address::from_string(bind_), port_);

    // Set socket options.
    socket_.open(listen_endpoint.protocol());
    // Set further socket options, these are not part of asio so we have
    // to use lower level APIs.  Get native handle.
    const int socket_fd = socket_.native_handle();

    int res = 0;
    int optval = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_RX_HARDWARE;

    // Enable time stamping on socket.
    res = setsockopt(socket_fd, SOL_SOCKET, SO_TIMESTAMPNS, &optval,
                     sizeof(optval));
    if (res != 0) {
      RCLCPP_ERROR(get_logger(),
                   "Error setting socket option SO_TIMESTAMPNS: %s",
                   strerror(errno));
      throw std::runtime_error("Error setting socket option SO_TIMESTAMPNS");
    }

    RCLCPP_INFO(get_logger(), "Listening on %s:%d", bind.c_str(), port_);

    try {
      // Bind means; which ip/interface should we listen to.
      socket_.bind(listen_endpoint);
    } catch (boost::system::system_error &e) {
      RCLCPP_ERROR(get_logger(), "Error binding socket: %s", e.what());
      throw std::runtime_error("Error binding socket: " +
                               std::string(e.what()));
    }

    // Publisher of timestamped udp packets
    // Use sensor data.
    udp_packet_publisher_ = create_publisher<raf_interfaces::msg::UDPPacket>(
        "udp_packet", QOS_BACKLOG);

    // Prepare packet message
    packet_msg_.header.frame_id = frame_id;
    packet_msg_.data.reserve(MAX_BUFFER_SIZE);

    // Schedule first read. Other reads are scheduled from the
    // handle_read function.
    read_one_packet();

    // Take the lock. We need to make sure that the constructor
    // has returned before the thread function is allowed to
    // execute.  This also sets up a memory fence to ensure a
    // consistent view of the state from both threads.
    std::scoped_lock lock(io_mutex_);

    // Start the io_service run loop in a separate thread.
    io_thread_ = std::thread([&]() {
      io_context_.run();
      RCLCPP_DEBUG(get_logger(), "io_service stopped");
    });

    RCLCPP_INFO(get_logger(), "raf_udp_receiver_node started");
  }

  virtual ~UdpReceiverNode() {
    socket_.close();
    io_context_.stop();
    // io_context will return when there are no more actors
    io_thread_.join();
  };
};
