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
#include <map>
#include <fstream>
#include <nlohmann/json.hpp>

#include <arpa/inet.h>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using json = nlohmann::json;

class CanInterface : public rclcpp::Node
{
public:
  CanInterface()
  : Node("can_node"), sockfd_(-1)
  {
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    node_id_ = static_cast<uint32_t>(this->declare_parameter<int>("node_id", 0x100));
    std::string config_file = this->declare_parameter<std::string>("leg_groups_config", "");

    if (!init_can_socket()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize CAN socket on '%s'", can_interface_.c_str());
      throw std::runtime_error("CAN initialization failed");
    }

    // Load leg groups configuration if provided
    if (!config_file.empty()) {
      load_leg_groups(config_file);
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
    // Determine target node IDs based on leg_number
    std::vector<uint32_t> target_nodes;
    
    if (static_cast<int8_t>(msg->leg_number) < 0) {
      // Negative leg_number references a leg group
      int group_id = static_cast<int8_t>(msg->leg_number);  // Directly use as group_id
      if (leg_groups_.find(group_id) != leg_groups_.end()) {
        target_nodes = leg_groups_[group_id];
        RCLCPP_INFO(this->get_logger(), "Leg group %d contains %zu node(s)", group_id, target_nodes.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "Leg group %d not found in configuration", group_id);
        return;
      }
    } else {
      // Positive leg_number maps to a single node: node_id_ + leg_number
      target_nodes.push_back(node_id_ + msg->leg_number);
    }

    // Pack the message into a raw byte payload for ISO-TP.
    std::vector<uint8_t> payload;

    // Helper to append arbitrary POD bytes
    auto append_bytes = [&payload](const void* data, size_t len) {
      const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
      payload.insert(payload.end(), p, p + len);
    };

    // Basic header: command_type, leg_number, axis
    payload.push_back(static_cast<uint8_t>(msg->command_type));

    // Pack floats according to command type
    if (msg->command_type == can::msg::LegCommand::LINEAR) {
      append_bytes(&msg->x, sizeof(msg->x));
      append_bytes(&msg->y, sizeof(msg->y));
      append_bytes(&msg->z, sizeof(msg->z));
      append_bytes(&msg->speed, sizeof(msg->speed));
    } else if (msg->command_type == can::msg::LegCommand::RAPID) {
      append_bytes(&msg->x, sizeof(msg->x));
      append_bytes(&msg->y, sizeof(msg->y));
      append_bytes(&msg->z, sizeof(msg->z));
    } else if (msg->command_type == can::msg::LegCommand::SINGLE_AXIS) {
      append_bytes(&msg->axis, sizeof(msg->axis));
      append_bytes(&msg->position, sizeof(msg->position));
    } else {
      RCLCPP_WARN(this->get_logger(), "Received LegCommand with unknown command_type: %d", msg->command_type);
    }

    if (payload.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty LegCommand");
      return;
    }

    if (payload.size() > 256U) {
      RCLCPP_WARN(this->get_logger(), "Packed payload too large (%zu bytes); maximum supported is 256 bytes", payload.size());
      return;
    }

    // Send to all target nodes
    for (uint32_t node_id : target_nodes) {
      if (!send_isotp(node_id, payload)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to send ISO-TP command to node 0x%X", node_id);
      } else {
        RCLCPP_INFO(this->get_logger(), "Sent ISO-TP command with payload size %zu bytes to node 0x%X", payload.size(), node_id);
      }
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

  void load_leg_groups(const std::string& config_file)
  {
    try {
      std::ifstream file(config_file);
      if (!file.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Could not open leg groups config file: %s", config_file.c_str());
        return;
      }

      json config = json::parse(file);
      
      if (config.contains("leg_groups") && config["leg_groups"].is_array()) {
        // Each index in the array maps to a group ID: index 0 -> -1, index 1 -> -2, etc.
        for (size_t i = 0; i < config["leg_groups"].size(); ++i) {
          auto& node_ids = config["leg_groups"][i];
          if (node_ids.is_array()) {
            std::vector<uint32_t> nodes;
            for (auto& node_val : node_ids) {
              if (node_val.is_number()) {
                nodes.push_back(node_id_ + static_cast<uint32_t>(node_val));
              }
            }
            int group_id = -(static_cast<int>(i) + 1); // Group IDs are negative, starting from -1
            leg_groups_[group_id] = nodes;
            RCLCPP_INFO(this->get_logger(), "Loaded leg group %d with %zu node(s)", group_id, nodes.size());
          }
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Config file does not contain 'leg_groups' array");
      }
    } catch (const json::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "JSON parsing error: %s", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading leg groups config: %s", e.what());
    }
  }

  int sockfd_;
  std::string can_interface_;
  uint32_t node_id_;
  rclcpp::Subscription<can::msg::LegCommand>::SharedPtr command_sub_;
  std::map<int, std::vector<uint32_t>> leg_groups_;  // Maps negative group IDs to lists of node IDs
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