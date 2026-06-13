#include "oculus_bridge/oculus_packet_decoder.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <optional>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace oculus_bridge
{

namespace
{

std::string endpoint_string(const sockaddr_in & remote_addr)
{
  char ip[INET_ADDRSTRLEN] = {0};
  inet_ntop(AF_INET, &remote_addr.sin_addr, ip, sizeof(ip));
  return std::string(ip) + ":" + std::to_string(ntohs(remote_addr.sin_port));
}

std::string ip_to_string(uint32_t ip)
{
  in_addr addr{};
  addr.s_addr = ip;
  char ip_text[INET_ADDRSTRLEN] = {0};
  inet_ntop(AF_INET, &addr, ip_text, sizeof(ip_text));
  return std::string(ip_text);
}

std::string mac_to_string(const std::vector<uint8_t> & mac)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < mac.size(); ++i) {
    if (i != 0) {
      oss << ":";
    }
    oss << std::setw(2) << static_cast<int>(mac[i]);
  }
  return oss.str();
}

std::string ping_json(const DecodedPing & ping)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss
    << "{"
    << "\"source\":\"" << ping.source << "\","
    << "\"version\":" << ping.version << ","
    << "\"src_device_id\":" << ping.src_device_id << ","
    << "\"ping_id\":" << ping.ping_id << ","
    << "\"status\":" << ping.status << ","
    << "\"frequency\":" << ping.frequency << ","
    << "\"temperature\":" << ping.temperature << ","
    << "\"pressure\":" << ping.pressure << ","
    << "\"heading\":" << ping.heading << ","
    << "\"pitch\":" << ping.pitch << ","
    << "\"roll\":" << ping.roll << ","
    << "\"range_resolution\":" << ping.range_resolution << ","
    << "\"n_ranges\":" << ping.n_ranges << ","
    << "\"n_beams\":" << ping.n_beams << ","
    << "\"data_size\":" << static_cast<int>(ping.data_size)
    << "}";
  return oss.str();
}

std::string status_json(const DecodedStatus & status)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss
    << "{"
    << "\"source\":\"" << status.source << "\","
    << "\"device_id\":" << status.device_id << ","
    << "\"device_type\":" << status.device_type << ","
    << "\"part_number\":" << status.part_number << ","
    << "\"status\":" << status.status << ","
    << "\"ip_addr\":\"" << ip_to_string(status.ip_addr) << "\","
    << "\"ip_mask\":\"" << ip_to_string(status.ip_mask) << "\","
    << "\"connected_ip_addr\":\"" << ip_to_string(status.connected_ip_addr) << "\","
    << "\"mac_addr\":\"" << mac_to_string(status.mac_address) << "\","
    << "\"pressure\":" << status.pressure << ","
    << "\"temperature0\":" << status.temperatures[0] << ","
    << "\"temperature1\":" << status.temperatures[1] << ","
    << "\"temperature2\":" << status.temperatures[2] << ","
    << "\"temperature3\":" << status.temperatures[3] << ","
    << "\"temperature4\":" << status.temperatures[4] << ","
    << "\"temperature5\":" << status.temperatures[5] << ","
    << "\"temperature6\":" << status.temperatures[6] << ","
    << "\"temperature7\":" << status.temperatures[7]
    << "}";
  return oss.str();
}

}  // namespace

class OculusBridgeNode : public rclcpp::Node
{
public:
  OculusBridgeNode()
  : Node("oculus_bridge_node")
  {
    sonar_address_ = declare_parameter<std::string>("sonar_address", "");
    sonar_data_port_ = declare_parameter<int>("sonar_data_port", 52100);
    status_bind_address_ = declare_parameter<std::string>("status_bind_address", "0.0.0.0");
    status_udp_port_ = declare_parameter<int>("status_udp_port", 52102);
    tcp_receive_buffer_size_ = declare_parameter<int>("tcp_receive_buffer_size", 200000);
    max_status_packet_size_ = declare_parameter<int>("max_status_packet_size", 2048);

    ping_raw_topic_ = declare_parameter<std::string>("ping_raw_topic", "oculus/ping/raw_packet");
    ping_image_topic_ = declare_parameter<std::string>("ping_image_topic", "oculus/ping/image");
    ping_bearings_topic_ = declare_parameter<std::string>("ping_bearings_topic", "oculus/ping/bearings");
    ping_metadata_topic_ = declare_parameter<std::string>("ping_metadata_topic", "oculus/ping/metadata");
    ping_point_cloud_topic_ = declare_parameter<std::string>("ping_point_cloud_topic", "oculus/ping/points");
    status_topic_ = declare_parameter<std::string>("status_topic", "oculus/status");

    auto_fire_ = declare_parameter<bool>("auto_fire", true);
    fire_interval_sec_ = declare_parameter<double>("fire_interval_sec", 1.0);
    fire_message_version_ = declare_parameter<int>("fire_message_version", 1);
    fire_frequency_mode_ = declare_parameter<int>("fire_frequency_mode", 2);
    fire_ping_rate_ = declare_parameter<int>("fire_ping_rate", 1);
    fire_network_speed_ = declare_parameter<int>("fire_network_speed", 0);
    fire_gamma_correction_ = declare_parameter<int>("fire_gamma_correction", 127);
    fire_flags_ = declare_parameter<int>("fire_flags", 0);
    fire_range_percent_or_meters_ = declare_parameter<double>("fire_range_percent_or_meters", 20.0);
    fire_gain_percent_ = declare_parameter<double>("fire_gain_percent", 50.0);
    fire_speed_of_sound_ = declare_parameter<double>("fire_speed_of_sound", 1500.0);
    fire_salinity_ = declare_parameter<double>("fire_salinity", 35.0);
    point_cloud_min_intensity_ = declare_parameter<int>("point_cloud_min_intensity", 32);
    point_cloud_range_stride_ = declare_parameter<int>("point_cloud_range_stride", 4);
    point_cloud_beam_stride_ = declare_parameter<int>("point_cloud_beam_stride", 2);
    point_cloud_frame_id_ = declare_parameter<std::string>("point_cloud_frame_id", "oculus_sonar");

    ping_raw_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(ping_raw_topic_, 10);
    ping_image_pub_ = create_publisher<sensor_msgs::msg::Image>(ping_image_topic_, 10);
    ping_bearings_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>(ping_bearings_topic_, 10);
    ping_metadata_pub_ = create_publisher<std_msgs::msg::String>(ping_metadata_topic_, 10);
    ping_point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ping_point_cloud_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    if (sonar_address_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Parameter 'sonar_address' is empty. TCP ping-result reception is disabled until it is set.");
    } else {
      data_socket_fd_ = open_tcp_socket(sonar_address_, sonar_data_port_);
      if (auto_fire_) {
        send_simple_fire();
        fire_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(fire_interval_sec_)),
          [this]() { send_simple_fire(); });
      }
      data_thread_ = std::thread(&OculusBridgeNode::data_loop, this);
      RCLCPP_INFO(
        get_logger(),
        "Connected to Oculus sonar TCP data stream at %s:%d.",
        sonar_address_.c_str(), sonar_data_port_);
    }

    status_socket_fd_ = open_udp_socket(status_bind_address_, status_udp_port_);
    status_thread_ = std::thread(&OculusBridgeNode::status_loop, this);
    RCLCPP_INFO(
      get_logger(),
      "Listening for Oculus status UDP packets on %s:%d.",
      status_bind_address_.c_str(), status_udp_port_);
  }

  ~OculusBridgeNode() override
  {
    running_.store(false);

    if (data_socket_fd_ >= 0) {
      shutdown(data_socket_fd_, SHUT_RDWR);
      close(data_socket_fd_);
      data_socket_fd_ = -1;
    }

    if (status_socket_fd_ >= 0) {
      shutdown(status_socket_fd_, SHUT_RDWR);
      close(status_socket_fd_);
      status_socket_fd_ = -1;
    }

    if (data_thread_.joinable()) {
      data_thread_.join();
    }
    if (status_thread_.joinable()) {
      status_thread_.join();
    }
  }

private:
  int open_tcp_socket(const std::string & address, int port)
  {
    if (port <= 0 || port > 65535) {
      throw std::runtime_error("Parameter 'sonar_data_port' must be between 1 and 65535.");
    }

    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) {
      throw std::runtime_error("Failed to create TCP socket: " + std::string(std::strerror(errno)));
    }

    int yes = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
    setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) {
      close(socket_fd);
      throw std::runtime_error("Invalid sonar address: " + address);
    }

    if (connect(socket_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      const std::string message = "Failed to connect TCP socket: " + std::string(std::strerror(errno));
      close(socket_fd);
      throw std::runtime_error(message);
    }

    return socket_fd;
  }

  int open_udp_socket(const std::string & bind_address, int port)
  {
    if (port <= 0 || port > 65535) {
      throw std::runtime_error("Parameter 'status_udp_port' must be between 1 and 65535.");
    }

    int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
      throw std::runtime_error("Failed to create UDP socket: " + std::string(std::strerror(errno)));
    }

    int reuse = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
      const std::string message = "Failed to set SO_REUSEADDR: " + std::string(std::strerror(errno));
      close(socket_fd);
      throw std::runtime_error(message);
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, bind_address.c_str(), &addr.sin_addr) != 1) {
      close(socket_fd);
      throw std::runtime_error("Invalid status bind address: " + bind_address);
    }

    if (bind(socket_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      const std::string message = "Failed to bind UDP socket: " + std::string(std::strerror(errno));
      close(socket_fd);
      throw std::runtime_error(message);
    }

    return socket_fd;
  }

  void data_loop()
  {
    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(static_cast<size_t>(tcp_receive_buffer_size_));
    std::vector<uint8_t> read_buffer(static_cast<size_t>(tcp_receive_buffer_size_));

    while (rclcpp::ok() && running_.load() && data_socket_fd_ >= 0) {
      const ssize_t bytes_read = recv(data_socket_fd_, read_buffer.data(), read_buffer.size(), 0);
      if (bytes_read == 0) {
        RCLCPP_WARN(get_logger(), "Oculus TCP data stream closed by peer.");
        break;
      }
      if (bytes_read < 0) {
        if (!running_.load()) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "TCP recv failed: %s", std::strerror(errno));
        continue;
      }

      rx_buffer.insert(rx_buffer.end(), read_buffer.begin(), read_buffer.begin() + bytes_read);
      process_tcp_buffer(rx_buffer);
    }
  }

  void send_simple_fire()
  {
    if (data_socket_fd_ < 0) {
      return;
    }

    std::vector<uint8_t> packet;
    packet.reserve(
      fire_message_version_ == 2 ? sizeof(OculusSimpleFireMessage2) : sizeof(OculusSimpleFireMessage));

    if (fire_message_version_ == 2) {
      OculusSimpleFireMessage2 fire{};
      fire.head.oculus_id = kOculusCheckId;
      fire.head.msg_id = static_cast<uint16_t>(OculusMessageType::kSimpleFire);
      fire.head.msg_version = 2;
      fire.head.payload_size = sizeof(OculusSimpleFireMessage2) - sizeof(OculusMessageHeader);
      fire.master_mode = static_cast<uint8_t>(fire_frequency_mode_);
      fire.ping_rate = static_cast<uint8_t>(fire_ping_rate_);
      fire.network_speed = static_cast<uint8_t>(fire_network_speed_);
      fire.gamma_correction = static_cast<uint8_t>(fire_gamma_correction_);
      fire.flags = static_cast<uint8_t>(fire_flags_);
      fire.range_percent = fire_range_percent_or_meters_;
      fire.gain_percent = fire_gain_percent_;
      fire.speed_of_sound = fire_speed_of_sound_;
      fire.salinity = fire_salinity_;

      packet.resize(sizeof(fire));
      std::memcpy(packet.data(), &fire, sizeof(fire));
    } else {
      OculusSimpleFireMessage fire{};
      fire.head.oculus_id = kOculusCheckId;
      fire.head.msg_id = static_cast<uint16_t>(OculusMessageType::kSimpleFire);
      fire.head.msg_version = 1;
      fire.head.payload_size = sizeof(OculusSimpleFireMessage) - sizeof(OculusMessageHeader);
      fire.master_mode = static_cast<uint8_t>(fire_frequency_mode_);
      fire.ping_rate = static_cast<uint8_t>(fire_ping_rate_);
      fire.network_speed = static_cast<uint8_t>(fire_network_speed_);
      fire.gamma_correction = static_cast<uint8_t>(fire_gamma_correction_);
      fire.flags = static_cast<uint8_t>(fire_flags_);
      fire.range = fire_range_percent_or_meters_;
      fire.gain_percent = fire_gain_percent_;
      fire.speed_of_sound = fire_speed_of_sound_;
      fire.salinity = fire_salinity_;

      packet.resize(sizeof(fire));
      std::memcpy(packet.data(), &fire, sizeof(fire));
    }

    std::lock_guard<std::mutex> lock(write_mutex_);
    const ssize_t sent = send(data_socket_fd_, packet.data(), packet.size(), 0);
    if (sent < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to send SimpleFire request: %s", std::strerror(errno));
    }
  }

  void process_tcp_buffer(std::vector<uint8_t> & buffer)
  {
    while (buffer.size() >= sizeof(OculusMessageHeader)) {
      OculusMessageHeader header{};
      std::memcpy(&header, buffer.data(), sizeof(OculusMessageHeader));

      if (header.oculus_id != kOculusCheckId) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Discarding TCP buffer due to invalid Oculus header.");
        buffer.clear();
        return;
      }

      const size_t packet_size = sizeof(OculusMessageHeader) + header.payload_size;
      if (buffer.size() < packet_size) {
        return;
      }

      std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + packet_size);
      buffer.erase(buffer.begin(), buffer.begin() + packet_size);

      std::string error;
      const auto decoded = decoder_.decode_ping_packet(
        packet, sonar_address_ + ":" + std::to_string(sonar_data_port_), error);
      if (!decoded) {
        if (header.msg_id != static_cast<uint16_t>(OculusMessageType::kUserConfig) &&
          header.msg_id != static_cast<uint16_t>(OculusMessageType::kDummy))
        {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Failed to decode TCP packet msg_id=0x%04x: %s", header.msg_id, error.c_str());
        }
        continue;
      }

      publish_ping(*decoded);
    }
  }

  void status_loop()
  {
    std::vector<uint8_t> buffer(static_cast<size_t>(max_status_packet_size_));

    while (rclcpp::ok() && running_.load() && status_socket_fd_ >= 0) {
      sockaddr_in remote_addr{};
      socklen_t remote_len = sizeof(remote_addr);
      const ssize_t received = recvfrom(
        status_socket_fd_,
        buffer.data(),
        buffer.size(),
        0,
        reinterpret_cast<sockaddr *>(&remote_addr),
        &remote_len);

      if (received < 0) {
        if (!running_.load()) {
          break;
        }
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "UDP recvfrom failed: %s", std::strerror(errno));
        continue;
      }

      std::vector<uint8_t> packet(buffer.begin(), buffer.begin() + received);
      std::string error;
      const auto decoded = decoder_.decode_status_packet(packet, endpoint_string(remote_addr), error);
      if (!decoded) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring invalid Oculus status packet: %s", error.c_str());
        continue;
      }

      publish_status(*decoded);
    }
  }

  void publish_ping(const DecodedPing & decoded)
  {
    std_msgs::msg::UInt8MultiArray raw_msg;
    raw_msg.data = decoded.raw_packet;
    ping_raw_pub_->publish(raw_msg);

    std_msgs::msg::Int16MultiArray bearings_msg;
    bearings_msg.data = decoded.bearings;
    ping_bearings_pub_->publish(bearings_msg);

    std_msgs::msg::String metadata_msg;
    metadata_msg.data = ping_json(decoded);
    ping_metadata_pub_->publish(metadata_msg);

    if (decoded.data_size == DataSizeType::k8Bit || decoded.data_size == DataSizeType::k16Bit) {
      sensor_msgs::msg::Image image_msg;
      image_msg.header.stamp = now();
      image_msg.header.frame_id = point_cloud_frame_id_;
      image_msg.width = decoded.n_beams;
      image_msg.height = decoded.n_ranges;
      image_msg.is_bigendian = false;
      image_msg.encoding = decoded.data_size == DataSizeType::k8Bit ? "mono8" : "mono16";
      image_msg.step = decoded.n_beams * (decoded.data_size == DataSizeType::k8Bit ? 1u : 2u);
      image_msg.data = decoded.image;
      ping_image_pub_->publish(image_msg);

      auto point_cloud = build_point_cloud(decoded);
      if (point_cloud.has_value()) {
        ping_point_cloud_pub_->publish(*point_cloud);
      }
    }
  }

  std::optional<sensor_msgs::msg::PointCloud2> build_point_cloud(const DecodedPing & decoded) const
  {
    if (decoded.bearings.empty() || decoded.image.empty()) {
      return std::nullopt;
    }

    const int range_stride = std::max(1, point_cloud_range_stride_);
    const int beam_stride = std::max(1, point_cloud_beam_stride_);
    const int intensity_threshold = std::max(0, point_cloud_min_intensity_);
    const bool is_8bit = decoded.data_size == DataSizeType::k8Bit;
    const bool is_16bit = decoded.data_size == DataSizeType::k16Bit;
    if (!is_8bit && !is_16bit) {
      return std::nullopt;
    }

    struct PointXYZI
    {
      float x;
      float y;
      float z;
      float intensity;
    };

    std::vector<PointXYZI> points;
    points.reserve((decoded.n_ranges / range_stride) * (decoded.n_beams / beam_stride));

    for (uint16_t r = 0; r < decoded.n_ranges; r += static_cast<uint16_t>(range_stride)) {
      const float radius = static_cast<float>((static_cast<double>(r) + 0.5) * decoded.range_resolution);

      for (uint16_t b = 0; b < decoded.n_beams; b += static_cast<uint16_t>(beam_stride)) {
        const size_t sample_index = static_cast<size_t>(r) * decoded.n_beams + b;
        float intensity = 0.0f;

        if (is_8bit) {
          if (sample_index >= decoded.image.size()) {
            continue;
          }
          intensity = static_cast<float>(decoded.image[sample_index]);
        } else {
          const size_t byte_index = sample_index * 2;
          if (byte_index + 1 >= decoded.image.size()) {
            continue;
          }
          uint16_t value = 0;
          std::memcpy(&value, decoded.image.data() + byte_index, sizeof(uint16_t));
          intensity = static_cast<float>(value);
        }

        if (intensity < static_cast<float>(intensity_threshold)) {
          continue;
        }

        const double bearing_rad =
          (static_cast<double>(decoded.bearings[b]) / 100.0) * M_PI / 180.0;

        PointXYZI point{};
        point.x = radius * static_cast<float>(std::cos(bearing_rad));
        point.y = radius * static_cast<float>(std::sin(bearing_rad));
        point.z = 0.0f;
        point.intensity = intensity;
        points.push_back(point);
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = point_cloud_frame_id_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.point_step = sizeof(PointXYZI);
    cloud.row_step = cloud.point_step * cloud.width;

    sensor_msgs::msg::PointField field;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;

    field.name = "x";
    field.offset = 0;
    cloud.fields.push_back(field);

    field.name = "y";
    field.offset = 4;
    cloud.fields.push_back(field);

    field.name = "z";
    field.offset = 8;
    cloud.fields.push_back(field);

    field.name = "intensity";
    field.offset = 12;
    cloud.fields.push_back(field);

    cloud.data.resize(points.size() * sizeof(PointXYZI));
    if (!points.empty()) {
      std::memcpy(cloud.data.data(), points.data(), cloud.data.size());
    }
    return cloud;
  }

  void publish_status(const DecodedStatus & decoded)
  {
    std_msgs::msg::String status_msg;
    status_msg.data = status_json(decoded);
    status_pub_->publish(status_msg);
  }

  std::atomic<bool> running_{true};
  int data_socket_fd_{-1};
  int status_socket_fd_{-1};
  std::thread data_thread_;
  std::thread status_thread_;
  std::mutex write_mutex_;

  std::string sonar_address_;
  int sonar_data_port_;
  std::string status_bind_address_;
  int status_udp_port_;
  int tcp_receive_buffer_size_;
  int max_status_packet_size_;

  std::string ping_raw_topic_;
  std::string ping_image_topic_;
  std::string ping_bearings_topic_;
  std::string ping_metadata_topic_;
  std::string ping_point_cloud_topic_;
  std::string status_topic_;
  bool auto_fire_;
  double fire_interval_sec_;
  int fire_message_version_;
  int fire_frequency_mode_;
  int fire_ping_rate_;
  int fire_network_speed_;
  int fire_gamma_correction_;
  int fire_flags_;
  double fire_range_percent_or_meters_;
  double fire_gain_percent_;
  double fire_speed_of_sound_;
  double fire_salinity_;
  int point_cloud_min_intensity_;
  int point_cloud_range_stride_;
  int point_cloud_beam_stride_;
  std::string point_cloud_frame_id_;
  rclcpp::TimerBase::SharedPtr fire_timer_;

  OculusPacketDecoder decoder_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr ping_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ping_image_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr ping_bearings_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ping_metadata_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ping_point_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

}  // namespace oculus_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<oculus_bridge::OculusBridgeNode>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("oculus_bridge_node"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
