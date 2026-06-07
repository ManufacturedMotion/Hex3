#include "can_interface.hpp"

#include <stdexcept>
#include <cstring>
#include <fstream>
#include <algorithm>

#include <arpa/inet.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ===================== Constructor =====================

CanInterface::CanInterface()
: Node("can_node"), sockfd_(-1)
{
  can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
  node_id_ = static_cast<uint32_t>(
    this->declare_parameter<int>("node_id", 0x100));

  std::string config_file =
    this->declare_parameter<std::string>("leg_groups_config", "");

  if (!init_can_socket()) {
    RCLCPP_FATAL(get_logger(),
      "Failed to initialize CAN socket on '%s'",
      can_interface_.c_str());
    throw std::runtime_error("CAN initialization failed");
  }

  if (!config_file.empty()) {
    load_leg_groups(config_file);
  }

  command_sub_ =
    this->create_subscription<hexapod_msgs::msg::LegCommand>(
      "/leg_commands",
      rclcpp::SensorDataQoS(),
      std::bind(
        &CanInterface::on_command_received,
        this,
        std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "can_node ready on interface '%s' target node ID 0x%X",
    can_interface_.c_str(),
    node_id_);
}

// ===================== Destructor =====================

CanInterface::~CanInterface()
{
  if (sockfd_ >= 0) {
    close(sockfd_);
  }
}

// ===================== CAN init =====================

bool CanInterface::init_can_socket()
{
  sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sockfd_ < 0) {
    RCLCPP_ERROR(get_logger(), "socket() failed: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

  if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(get_logger(), "ioctl failed: %s", std::strerror(errno));
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sockfd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(get_logger(), "bind failed: %s", std::strerror(errno));
    close(sockfd_);
    sockfd_ = -1;
    return false;
  }

  return true;
}

// ===================== ROS callback =====================

void CanInterface::on_command_received(
  const hexapod_msgs::msg::LegCommand::SharedPtr msg)
{
  std::vector<uint32_t> target_nodes;

  if (static_cast<int8_t>(msg->leg_number) < 0) {
    int group_id = static_cast<int8_t>(msg->leg_number);

    if (!leg_groups_.count(group_id)) {
      RCLCPP_WARN(get_logger(),
        "Leg group %d not found",
        group_id);
      return;
    }

    target_nodes = leg_groups_[group_id];
  } else {
    target_nodes.push_back(node_id_ + msg->leg_number);
  }

  std::vector<uint8_t> payload;

  auto append_bytes = [&payload](const void* data, size_t len) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    payload.insert(payload.end(), p, p + len);
  };

  payload.push_back(static_cast<uint8_t>(msg->command_type));

  if (msg->command_type == hexapod_msgs::msg::LegCommand::LINEAR) {
    append_bytes(&msg->x, sizeof(msg->x));
    append_bytes(&msg->y, sizeof(msg->y));
    append_bytes(&msg->z, sizeof(msg->z));
    append_bytes(&msg->speed, sizeof(msg->speed));
  }
  else if (msg->command_type == hexapod_msgs::msg::LegCommand::RAPID) {
    append_bytes(&msg->x, sizeof(msg->x));
    append_bytes(&msg->y, sizeof(msg->y));
    append_bytes(&msg->z, sizeof(msg->z));
  }
  else if (msg->command_type == hexapod_msgs::msg::LegCommand::SINGLE_AXIS) {
    append_bytes(&msg->axis, sizeof(msg->axis));
    append_bytes(&msg->position, sizeof(msg->position));
  }
  else {
    RCLCPP_WARN(get_logger(), "Unknown command_type: %d", msg->command_type);
    return;
  }

  if (payload.size() > 256U) {
    RCLCPP_WARN(get_logger(), "Payload too large: %zu", payload.size());
    return;
  }

  for (auto node_id : target_nodes) {
    if (!send_isotp(node_id, payload)) {
      RCLCPP_ERROR(get_logger(),
        "Failed sending ISO-TP to 0x%X",
        node_id);
    }
    else {
      RCLCPP_INFO(get_logger(),
        "Sent command to 0x%X: type=%d payload_size=%zu",
        node_id, msg->command_type, payload.size());
    }
  }
}

// ===================== CAN frame =====================

bool CanInterface::send_can_frame(
  uint32_t can_id,
  const uint8_t* data,
  uint8_t dlc)
{
  struct can_frame frame {};
  frame.can_id = can_id & CAN_SFF_MASK;
  frame.can_dlc = dlc;
  std::memcpy(frame.data, data, dlc);

  return write(sockfd_, &frame, sizeof(frame)) ==
         static_cast<ssize_t>(sizeof(frame));
}

// ===================== ISO-TP =====================

bool CanInterface::send_isotp(
  uint32_t node_id,
  const std::vector<uint8_t>& payload)
{
  const size_t len = payload.size();
  if (len > 256U) return false;

  if (len <= 7U) {
    uint8_t frame[8]{};
    frame[0] = (0x0 << 4) | (len & 0x0F);
    std::memcpy(&frame[1], payload.data(), len);
    return send_can_frame(node_id, frame, len + 1);
  }

  uint8_t ff[8]{};
  ff[0] = (0x1 << 4) | ((len >> 8) & 0x0F);
  ff[1] = len & 0xFF;
  std::memcpy(&ff[2], payload.data(), 6);

  if (!send_can_frame(node_id, ff, 8)) return false;

  size_t offset = 6;
  uint8_t seq = 1;

  while (offset < len) {
    uint8_t cf[8]{};
    cf[0] = (0x2 << 4) | (seq & 0x0F);

    size_t chunk = std::min<size_t>(7, len - offset);
    std::memcpy(&cf[1], payload.data() + offset, chunk);

    if (!send_can_frame(node_id, cf, chunk + 1)) return false;

    offset += chunk;
    seq = (seq + 1) & 0x0F;
    if (seq == 0) seq = 1;
  }

  return true;
}

// ===================== JSON config =====================

void CanInterface::load_leg_groups(const std::string& file)
{
  try {
    std::ifstream f(file);
    if (!f.is_open()) return;

    json cfg = json::parse(f);

    if (!cfg.contains("leg_groups")) return;

    for (size_t i = 0; i < cfg["leg_groups"].size(); i++) {
      std::vector<uint32_t> nodes;

      for (auto& v : cfg["leg_groups"][i]) {
        nodes.push_back(node_id_ + (uint32_t)v);
      }

      int group_id = -(int(i) + 1);
      leg_groups_[group_id] = nodes;
    }

  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to parse config");
  }
}

// ===================== main =====================

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<CanInterface>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"),
      "Exception: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}