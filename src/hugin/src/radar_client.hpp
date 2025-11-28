// Copyright (c) Sensrad 2023

#pragma once

#include <array>
#include <memory>
#include <optional>

#include <boost/algorithm/hex.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/system/error_code.hpp>

#include <arbe_raf/CommonBase.h>
#include <arbe_raf/Raf_Api.h>
#include <arbe_raf/Utils.h>

namespace asio = boost::asio;
using namespace std::chrono_literals;

namespace hugin {

// Timeout exceptions
class Timeout : public std::exception {};
class ConnectTimeout : Timeout {};
class ReadTimeout : Timeout {};
class WriteTimeout : Timeout {};
class StatusTimeout : Timeout {};
class Ack0Timeout : Timeout {};
class Ack1Timeout : Timeout {};

// Error exceptions
class Error : public std::exception {
public:
  Error(const std::string &msg) : msg_(msg) {}
  const char *what() const noexcept override { return msg_.c_str(); }

private:
  const std::string msg_;
};

class ConnectError : Error {
public:
  ConnectError(const std::string &msg) : Error(msg) {}
};

class ReadError : Error {
public:
  ReadError(const std::string &msg) : Error(msg) {}
};

class WriteError : Error {
public:
  WriteError(const std::string &msg) : Error(msg) {}
};

// Main radar client class
class RadarClient : public std::enable_shared_from_this<RadarClient> {

  TArbeApiMailBox mailbox_;
  asio::io_context io_context_;
  std::shared_ptr<asio::ip::tcp::socket> socket_;

  static constexpr int TIMEOUT_ACK = 500;         // ms
  static constexpr int TIMEOUT_CONNECTION = 2000; // ms

  static constexpr int RAF_API_MAX_PAYLOAD_SIZE = 256;

  // Set and assert some parameters used in 1.8.5
  static_assert(AZ_CDF_NUM_OF_THR_LEVELS == 16,
                "AZ_CDF_NUM_OF_THR_LEVELS changed from 16");

  struct __attribute__((packed)) RafApiPacket {
    // Data fields
    TRAF_API_Header header;
    std::uint8_t payload[RAF_API_MAX_PAYLOAD_SIZE];

    std::uint16_t prefix() const { return header.usPrefix; }

    EOutputType packet_type() const noexcept {
      // Expect packets to not have an invalid type.
      return static_cast<EOutputType>(header.usType);
    }

    // This packet is sent at radar startup.
    bool is_status_packet() const noexcept {
      return packet_type() == Status_Output;
    }

    // This is the "ack" packet type, sent twice in response to a
    // command.
    bool is_response_packet() const noexcept {
      return packet_type() == Response_Output;
    }

    // Returns the size of the payload in bytes.
    std::size_t payload_size() const noexcept {
      return header.unLength - sizeof(header);
    }

    // Returns the size of the packet in bytes.
    std::size_t size() const noexcept { return header.unLength; }

    std::uint32_t message_number() const noexcept {
      return header.unMessageNumber;
    }
  };

  // Return time since epoch in ms
  static unsigned long get_system_time_ms() {
    const auto now = std::chrono::system_clock::now();
    const auto now_ms =
        std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    const auto value = now_ms.time_since_epoch();
    return static_cast<unsigned long>(value.count());
  }

  // C trampoline
  static uint32_t send_command_(int32_t unMsgID, uint8_t *pucBuffer,
                                uint32_t unLen, void *user_data) {
    auto self = static_cast<RadarClient *>(user_data);
    std::vector<std::uint8_t> buffer(pucBuffer, pucBuffer + unLen);
    return self->send_command(unMsgID, buffer);
  };

  uint32_t send_command(int32_t unMsgID, const std::vector<uint8_t> &buffer) {

    if (not socket_) {
      throw std::runtime_error("Socket not connected");
    }

    std::cerr << "Sending command with message id: " << unMsgID << std::endl;

    size_t written =
        write_with_timeout(asio::buffer(buffer.data(), buffer.size()));
    assert(written == buffer.size());
    (void)written;

    // Wait for ack x 2
    try {
      const auto ack0 = read_packet();
      if (not ack0.is_response_packet()) {
        throw std::runtime_error("Ack0: expected response packet");
      }
    } catch (const ReadTimeout &) {
      throw Ack0Timeout();
    }

    try {
      const auto ack0 = read_packet();
      if (not ack0.is_response_packet()) {
        throw std::runtime_error("Ack0: expected response packet");
      }
    } catch (const ReadTimeout &) {
      throw Ack1Timeout();
    }

    // Return code is not used by the API.
    return 0;
  };

  // Reads a packet from the socket.
  RafApiPacket read_packet() {
    RafApiPacket packet;

    std::size_t bytes_read = 0;

    // Read header (since we don't know the size of the payload)
    bytes_read =
        read_with_timeout(asio::buffer(&packet.header, sizeof(packet.header)));

    assert(bytes_read == sizeof(packet.header));
    (void)bytes_read;
    // Read payload (since we now the size of the payload)
    bytes_read =
        read_with_timeout(asio::buffer(packet.payload, packet.payload_size()));
    assert(bytes_read == packet.payload_size());

    if (packet.prefix() != 0xa55a) {
      throw std::runtime_error("Invalid packet prefix");
    }

    // Convert payload to hex string
    std::stringstream ss;
    ss << std::hex << std::setw(2) << std::setfill('0');
    for (std::size_t i = 0; i < packet.payload_size(); ++i) {
      ss << static_cast<int>(packet.payload[i]);
    }

    switch (packet.packet_type()) {
    case Status_Output:
      std::cerr << "Received status packet: ";
      break;
    case Response_Output:
      std::cerr << "Received response packet: ";
      break;
    default:
      std::cerr << "Received other packet: ";
      break;
    }
    // Rest of info
    std::cerr << packet.header.unMessageNumber << packet.payload_size() << " "
              << ss.str() << std::endl;

    return packet;
  }

  // Synchronous write with timeout
  template <class MutableBufferSequence>
  std::size_t write_with_timeout(const MutableBufferSequence &buffers,
                                 std::uint32_t timeout_ms = TIMEOUT_ACK) {

    std::optional<boost::system::error_code> write_result;
    std::optional<boost::system::error_code> timer_result;
    std::size_t bytes_transferred = 0;
    asio::deadline_timer timer(io_context_);

    timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    timer.async_wait([&timer_result](const boost::system::error_code &error) {
      timer_result = error;
    });

    socket_->async_write_some(
        buffers, [&write_result, &bytes_transferred](
                     const boost::system::error_code &error, size_t size) {
          write_result = error;
          bytes_transferred = size;
        });

    io_context_.restart();

    // Execute handlers
    while (io_context_.run_one()) {
      // Cancel the remaining handler
      if (write_result) {
        timer.cancel();
      } else if (timer_result) {
        socket_->cancel();
      }
    }

    if (*timer_result == boost::system::errc::success) {
      std::cerr << "Write timeout" << timer_result->message() << std::endl;
      std::cerr << "Write timeout" << std::endl;
    }

    // Both handlers have executed.
    if (not *timer_result) {
      throw WriteTimeout();
    } else if (*write_result) {
      throw WriteError(write_result->message());
    }

    return bytes_transferred;
  }

  // Synchronous read with timeout
  template <class MutableBufferSequence>
  std::size_t read_with_timeout(const MutableBufferSequence &buffers,
                                std::uint32_t timeout_ms = TIMEOUT_ACK) {

    std::optional<boost::system::error_code> read_result;
    std::optional<boost::system::error_code> timer_result;
    std::size_t bytes_transferred = 0;
    asio::deadline_timer timer(io_context_);

    timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    timer.async_wait([&timer_result](const boost::system::error_code &error) {
      timer_result = error;
    });

    socket_->async_read_some(
        buffers, [&read_result, &bytes_transferred](
                     const boost::system::error_code &error, size_t size) {
          read_result = error;
          bytes_transferred = size;
        });

    io_context_.restart();

    // Execute handlers
    while (io_context_.run_one()) {
      // Cancel the remaining handler
      if (read_result) {
        timer.cancel();
      } else if (timer_result) {
        socket_->cancel();
      }
    }

    // Both handlers have executed.
    if (not *timer_result) {
      throw ReadTimeout();
    } else if (*read_result) {
      throw ReadError(read_result->message());
    }

    return bytes_transferred;
  }

  // Synchronous connect with timeout
  void connect_with_timeout(std::string &ip, std::uint16_t port,
                            std::uint32_t timeout_ms = TIMEOUT_CONNECTION) {

    std::optional<boost::system::error_code> connect_result;
    std::optional<boost::system::error_code> timer_result;

    asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string(ip), port);

    socket_.reset(new asio::ip::tcp::socket(io_context_));
    asio::deadline_timer timer(io_context_);

    timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    timer.async_wait([&timer_result](const boost::system::error_code &error) {
      timer_result = error;
    });

    socket_->async_connect(
        endpoint, [&connect_result](const boost::system::error_code &error) {
          connect_result = error;
        });

    io_context_.restart();

    // Execute handlers
    while (io_context_.run_one()) {

      if (connect_result) {
        timer.cancel();
      } else if (timer_result) {
        socket_->cancel();
      }
    }
    // Both handlers have executed.
    if (not *timer_result) {
      throw ConnectTimeout();
    } else if (*connect_result) {
      throw ConnectError(connect_result->message());
    }
  }

public:
  typedef unsigned long (*time_function_ms)();

  RadarClient(time_function_ms time_func = get_system_time_ms) {
    mailbox_.sysCfg_GetSystemTimeMs = time_func;
    mailbox_.sendCmd = send_command_;
    mailbox_.unMessageNumber = 0;
    mailbox_.user_data = this;
  }

  virtual ~RadarClient() {
    if (socket_) {
      disconnect();
    }
  }

  void connect(std::string &ip, std::uint16_t port) {
    connect_with_timeout(ip, port);
    const auto status = read_packet();
    if (not status.is_status_packet()) {
      throw std::runtime_error("Expected status packet");
    }
  }

  void disconnect() {
    socket_->shutdown(asio::ip::tcp::socket::shutdown_both);
    socket_->close();
    socket_.reset();
  }

  // Radar control functions, these wrap the Arbe API functions.
  void radar_start_tx() {
    TStartTxInfo start_tx_info;
    RAF_API_RdrCtrl_StartTx(&mailbox_, &start_tx_info);
  }

  void radar_stop_tx() {
    TStopInfo stop_tx_info;
    RAF_API_RdrCtrl_StopTx(&mailbox_, &stop_tx_info);
  }

  void radar_sync_time_ms(uint64_t time_ms) {
    TSetTimeInfo time_info;

    time_info.unInitateTimeLsb = (uint32_t)((time_ms & 0x00000000ffffffff));
    time_info.unInitateTimeMsb =
        (uint32_t)((time_ms & 0xffffffff00000000) >> 32);
    // Send time
    RAF_API_SysCfg_SetTime(&mailbox_, time_info);
  }

  void radar_set_active_seq(ESeuqenceType sequence_type) {
    TSelectActiveSeqInfo active_seq_info;

    active_seq_info.eSequenceType = sequence_type;
    // Send
    RAF_API_RdrCtrl_SetActiveSeq(&mailbox_, &active_seq_info);
  }

  // Not sure if the defaults are correct
  void radar_set_thresholds(const float bias_4d = 0.0,
                            const float static_threshold = 5.0,
                            const float dynamic_azimuth_sensitivity = 20.0,
                            const float dynamic_elevation_sensitivity = 5.0) {

    const float NOISE_LEVEL = 60;

    // Copied from Arbe's example code.
    TSetThresholdsInfo radar_thresholds_info;

    radar_thresholds_info.opcode = SetStaticAndDynamicThresholds;

    float threshold4d = static_threshold + NOISE_LEVEL;
    float threshold3d = threshold4d - bias_4d;

    const float scale = 16.f / 3.f;

    radar_thresholds_info.aunParams[0] =
        static_cast<uint32_t>(threshold3d * scale);
    radar_thresholds_info.aunParams[1] =
        static_cast<uint32_t>(threshold4d * scale);
    radar_thresholds_info.aunParams[2] =
        static_cast<uint32_t>(dynamic_azimuth_sensitivity * scale);
    radar_thresholds_info.aunParams[3] =
        static_cast<uint32_t>(dynamic_elevation_sensitivity * scale);
    // Send
    RAF_API_RdrCtrl_SetThresholds(&mailbox_, &radar_thresholds_info);
  }

  void radar_set_cfar_mode(bool cfar_3d_enabled, bool cfar_4d_enabled,
                           bool send_metadata) {
    TCFARInfo cfar_info{};

    cfar_info.unCFARCoarseOn = cfar_3d_enabled;
    cfar_info.unCFARFineOn = cfar_4d_enabled;
    cfar_info.unSendMetadata = send_metadata;
    // Send
    RAF_API_SysCfg_CFAR(&mailbox_, &cfar_info);
  }

  // Available from Arbe Firmware 1.8.1
  void radar_set_adt(bool enable) {
    TSetAdtInfo adt_info{};

    adt_info.unAdtSet = enable;
    // Send
    RAF_API_SysCfg_SetAdt(&mailbox_, &adt_info);
  }

  // Available from Arbe Firmware 1.8.5
  void radar_set_local_max(bool enable) {
    TSetLocalMaxInfo local_max_info{};

    local_max_info.unSetReset = enable;
    // Send
    RAF_API_SysCfg_LocalMax(&mailbox_, &local_max_info);
  }

  // Available from Arbe Firmware 1.8.5
  void radar_set_az_cdf_thr_levels(
      uint32_t unFrameType,
      const std::array<uint16_t, AZ_CDF_NUM_OF_THR_LEVELS> &thr_levels) {
    TAzCdfThrInfo az_cdf_thr_info{};

    az_cdf_thr_info.unFrameType = unFrameType;
    std::copy(thr_levels.begin(), thr_levels.end(),
              az_cdf_thr_info.pusThrLevels);
    // Send
    RAF_API_SysCfg_SetAzCdfThrLvls(&mailbox_, &az_cdf_thr_info);
  }

  // Available from Arbe Firmware 1.8.5
  void radar_set_range_hist_config(uint32_t start_offset,
                                   uint32_t increment_level) {
    TRangeHistConfigInfo range_hist_info{};

    range_hist_info.unThrStartOffset = start_offset;
    range_hist_info.unPowerIncrementLevel = increment_level;
    // Send
    RAF_API_SysCfg_SetRangeHistConfig(&mailbox_, &range_hist_info);
  }

  // Available from Arbe Firmware 1.8.5
  void radar_set_hist_output(u_int32_t enable_az_hist,
                             uint32_t enable_range_hist) {
    THistOutputInfo hist_output_info{};

    hist_output_info.unAzimuthHistOutputEnable = enable_az_hist;
    hist_output_info.unRangeHistOutputEnable = enable_range_hist;
    // Send
    RAF_API_SysCfg_SetHistogramOutput(&mailbox_, &hist_output_info);
  }

  // Available from Arbe Firmware 1.8.5
  void radar_set_spot_rsl(bool enable_spot, bool enable_rsl) {
    TDspSpotRslOnOffInfo spot_rsl_info{};

    spot_rsl_info.unIsSpotOn = enable_spot;
    spot_rsl_info.unIsRslOn = enable_rsl;
    // Send
    RAF_API_SysCfg_DspSetSpotRslOnOff(&mailbox_, &spot_rsl_info);
  }

  // Available from Arbe Firmware 1.8.5. We only set the back_off parameter to
  // mimic the Arbe gui implementation
  void radar_set_rsl_params(uint32_t back_off) {
    TDspSetRslParamsInfo rsl_params_info{};

    rsl_params_info.unRslBackOff = back_off;

    // Send
    RAF_API_SysCfg_DspSetRslParams(&mailbox_, &rsl_params_info);
  }

  // Set base-frequency
  bool radar_set_base_frequency(const float base_freq_khz) {
    if (base_freq_khz / 1e6 > 76.0 && base_freq_khz / 1e6 < 81.0) {
      TCtrlFrameControlInfo freqInfo{};

      freqInfo.unBandwidth = std::numeric_limits<uint32_t>::max();
      freqInfo.unBaseFreq = static_cast<uint32_t>(base_freq_khz);

      for (int type = 0; type < EFrameTypeUser::FrameTypeLast; type++) {

        freqInfo.unFrameType = static_cast<uint32_t>(type);
        RAF_API_RdrCtrl_SetFrameControlData(&mailbox_, &freqInfo);
      }
      return true;
    } else {
      return false;
    }
  }

  // Configure NTC
  void radar_set_ntc_mode(bool ntc_3d_enabled, bool ntc_4d_enabled,
                          uint32_t ntc_percentage, bool send_metadata) {

    TNtcInfo ntc_info{};

    ntc_info.unNTCCoarseOn = ntc_3d_enabled;
    ntc_info.unNTCFineOn = ntc_4d_enabled;
    ntc_info.unNTCPercentage = ntc_percentage;
    ntc_info.unSendMetadata = send_metadata;
    // Send
    RAF_API_SysCfg_NTC(&mailbox_, &ntc_info);
  }

  void radar_set_packet_format(bool phase_enabled, uint32_t packet_format) {

    TConfigPacketFormatInfo config_packet_format_info{};

    uint8_t phase_config = phase_enabled;
    // 3 for 80 Bit uncompressed(48Bit), 2 for 96 Bit uncompressed(56Bits - with
    // phase)
    config_packet_format_info.unPacketFormat = phase_config + packet_format;
    // Send
    RAF_API_SysCfg_ConfigPacketFormat(&mailbox_, &config_packet_format_info);
  }
};

} // namespace hugin
