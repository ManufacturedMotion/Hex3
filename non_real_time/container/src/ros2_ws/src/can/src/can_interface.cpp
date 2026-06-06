// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world can package\n");
//   return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "can/msg/leg_command.hpp"
#include <vector>
#include <algorithm>

#include <arpa/inet.h>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

class CanInterface : public rclcpp::Node
{
public:
  CanInterface()
  : Node("can_node"), sockfd_(-1)
  {
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    node_id_ = static_cast<uint32_t>(this->declare_parameter<int>("node_id", 0x100));

    if (!init_can_socket()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize CAN socket on '%s'", can_interface_.c_str());
      throw std::runtime_error("CAN initialization failed");
    }

    command_sub_ = this->create_subscription<can::msg::LegCommand>(
      "leg_command",
      rclcpp::SensorDataQoS(),
      std::bind(&CanInterface::on_command_received, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "can_node ready on interface '%s' target node ID 0x%X", can_interface_.c_str(), node_id_);
  }

  ~CanInterface() override
  {
    if (sockfd_ >= 0) {
      close(sockfd_);
    }
  }

private:
  bool init_can_socket()
  {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "socket() failed: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "ioctl(SIOCGIFINDEX) failed: %s", std::strerror(errno));
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "bind() failed: %s", std::strerror(errno));
      close(sockfd_);
      sockfd_ = -1;
      return false;
    }

    return true;
  }

  void on_command_received(const can::msg::LegCommand::SharedPtr msg)
  {
    // Pack the message into a raw byte payload for ISO-TP.
    std::vector<uint8_t> payload;

    // Helper to append arbitrary POD bytes
    auto append_bytes = [&payload](const void* data, size_t len) {
      const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
      payload.insert(payload.end(), p, p + len);
    };

    // Basic header: command_type, leg_number, axis
    payload.push_back(static_cast<uint8_t>(msg->command_type));
    payload.push_back(static_cast<uint8_t>(msg->leg_number));
    payload.push_back(static_cast<uint8_t>(msg->axis));

    // Pack floats according to command type. Unused fields are still present
    // so message layout is fixed and predictable.
    append_bytes(&msg->x, sizeof(msg->x));
    append_bytes(&msg->y, sizeof(msg->y));
    append_bytes(&msg->z, sizeof(msg->z));
    append_bytes(&msg->speed, sizeof(msg->speed));
    append_bytes(&msg->position, sizeof(msg->position));

    if (payload.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty LegCommand");
      return;
    }

    if (payload.size() > 256U) {
      RCLCPP_WARN(this->get_logger(), "Packed payload too large (%zu bytes); maximum supported is 256 bytes", payload.size());
      return;
    }

    if (!send_isotp(node_id_, payload)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send ISO-TP command");
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Sent ISO-TP command with payload size %zu bytes", payload.size());
    }
  }

  bool send_can_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc)
  {
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id & CAN_SFF_MASK;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);

    ssize_t bytes = write(sockfd_, &frame, sizeof(frame));
    if (bytes != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_ERROR(this->get_logger(), "CAN write failed: %s", std::strerror(errno));
      return false;
    }

    return true;
  }

  bool send_isotp(uint32_t node_id, const std::vector<uint8_t>& payload)
  {
    const size_t payload_len = payload.size();
    if (payload_len > 256U) {
      RCLCPP_ERROR(this->get_logger(), "ISO-TP payload exceeds supported length: %zu", payload_len);
      return false;
    }

    if (payload_len <= 7U) {
      uint8_t frame[8];
      std::memset(frame, 0, sizeof(frame));
      frame[0] = static_cast<uint8_t>((0x0 << 4) | (payload_len & 0x0F));
      std::memcpy(&frame[1], payload.data(), payload_len);
      return send_can_frame(node_id, frame, static_cast<uint8_t>(payload_len + 1U));
    }

    uint8_t first_frame[8];
    std::memset(first_frame, 0, sizeof(first_frame));
    first_frame[0] = static_cast<uint8_t>((0x1 << 4) | ((payload_len >> 8) & 0x0F));
    first_frame[1] = static_cast<uint8_t>(payload_len & 0xFF);
    std::memcpy(&first_frame[2], payload.data(), 6);

    if (!send_can_frame(node_id, first_frame, 8)) {
      return false;
    }

    size_t offset = 6;
    uint8_t seq = 1;

    while (offset < payload_len) {
      uint8_t frame[8];
      std::memset(frame, 0, sizeof(frame));
      frame[0] = static_cast<uint8_t>((0x2 << 4) | (seq & 0x0F));

      const size_t copy_len = std::min<size_t>(7U, payload_len - offset);
      std::memcpy(&frame[1], payload.data() + offset, copy_len);

      if (!send_can_frame(node_id, frame, static_cast<uint8_t>(copy_len + 1U))) {
        return false;
      }

      offset += copy_len;
      seq = static_cast<uint8_t>((seq + 1) & 0x0F);
      if (seq == 0) {
        seq = 1;
      }
    }

    return true;
  }

  int sockfd_;
  std::string can_interface_;
  uint32_t node_id_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr command_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<CanInterface>();
    rclcpp::spin(node);
  } catch (const std::exception& ex) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", ex.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}