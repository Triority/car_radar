#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <string>

// Helper to append any basic data type to a byte buffer
template<typename T>
void append_to_buffer(std::vector<uint8_t>& buffer, const T& value) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

// Our reliable manual serializer for PointCloud2
std::vector<uint8_t> serialize_pointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<uint8_t> buffer;
    buffer.reserve(512 + msg->data.size()); // Pre-allocate memory

    // Serialize metadata
    append_to_buffer(buffer, msg->height);
    append_to_buffer(buffer, msg->width);
    append_to_buffer(buffer, static_cast<uint8_t>(msg->is_bigendian));
    append_to_buffer(buffer, msg->point_step);
    append_to_buffer(buffer, msg->row_step);
    append_to_buffer(buffer, static_cast<uint8_t>(msg->is_dense));

    // Serialize fields
    uint32_t num_fields = msg->fields.size();
    append_to_buffer(buffer, num_fields);
    for (const auto& field : msg->fields) {
        uint8_t name_len = field.name.length();
        append_to_buffer(buffer, name_len);
        buffer.insert(buffer.end(), field.name.begin(), field.name.end());
        append_to_buffer(buffer, field.offset);
        append_to_buffer(buffer, field.datatype);
        append_to_buffer(buffer, field.count);
    }

    // Serialize point data
    uint32_t data_len = msg->data.size();
    append_to_buffer(buffer, data_len);
    buffer.insert(buffer.end(), msg->data.begin(), msg->data.end());

    return buffer;
}


class PointCloudTCPSender : public rclcpp::Node
{
public:
    PointCloudTCPSender() : Node("pointcloud_tcp_sender")
    {
        this->declare_parameter<std::string>("dest_ip", "127.0.0.1");
        this->declare_parameter<int>("dest_port", 12345);
        this->get_parameter("dest_ip", dest_ip_);
        this->get_parameter("dest_port", dest_port_);

        // 1. Create a TCP socket (SOCK_STREAM)
        if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed");
            return;
        }

        struct sockaddr_in dest_addr_;
        memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(dest_port_);
        inet_pton(AF_INET, dest_ip_.c_str(), &dest_addr_.sin_addr);

        // 2. Connect to the server
        RCLCPP_INFO(this->get_logger(), "Connecting to server %s:%d...", dest_ip_.c_str(), dest_port_);
        if (connect(sockfd_, (struct sockaddr *)&dest_addr_, sizeof(dest_addr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection Failed. Is the ROS1 receiver running?");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connection successful.");

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/oden_1/extended_point_cloud", // <-- Use your actual topic here
            10,
            std::bind(&PointCloudTCPSender::topic_callback, this, std::placeholders::_1));
    }

    ~PointCloudTCPSender()
    {
        close(sockfd_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<uint8_t> serialized_data = serialize_pointcloud2(msg);
        uint32_t data_size = serialized_data.size();

        // 3. Send data over TCP.
        // Protocol: First send 4 bytes (uint32_t) of size, then send the actual data.
        ssize_t bytes_sent = send(sockfd_, &data_size, sizeof(data_size), 0);
        if (bytes_sent <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data size. Connection may be closed.");
            return;
        }

        bytes_sent = send(sockfd_, serialized_data.data(), data_size, 0);
        if (bytes_sent < 0 || (uint32_t)bytes_sent != data_size) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send full data payload.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sent a serialized PointCloud2 message of %u bytes.", data_size);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    int sockfd_;
    std::string dest_ip_;
    int dest_port_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTCPSender>());
    rclcpp::shutdown();
    return 0;
}
