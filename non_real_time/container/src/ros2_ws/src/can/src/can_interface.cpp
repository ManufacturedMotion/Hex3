#include "can_interface.hpp"

#include <stdexcept>
#include <cstring>
#include <fstream>
#include <algorithm>

#include <arpa/inet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can/isotp.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

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

  for (uint8_t leg_num = 0; leg_num < 6; leg_num++) {
    uint32_t node_id = node_id_ + leg_num;
    if (!create_isotp_socket(node_id)) {
      RCLCPP_ERROR(get_logger(),
        "Failed to create ISO-TP socket for node ID 0x%X",
        node_id);
    }
    else {
      RCLCPP_INFO(get_logger(),
        "Created ISO-TP socket for node ID 0x%X",
        node_id);
    }
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

  scheduler_running_ = true;

  scheduler_thread_ =
    std::thread(
        &CanInterface::scheduler_loop,
        this);

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

  scheduler_running_ = false;

  if (scheduler_thread_.joinable())
  {
      scheduler_thread_.join();
  }
}

// ===================== CAN init =====================

bool CanInterface::init_can_socket()
{
  sockfd_ = socket(AF_CAN, SOCK_DGRAM, CAN_ISOTP);
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

bool CanInterface::create_isotp_socket(uint32_t node_id)
{
    int sock = socket(AF_CAN, SOCK_DGRAM, CAN_ISOTP);
    if (sock < 0)
    {
        RCLCPP_ERROR(get_logger(), "ISO-TP socket failed: %s", strerror(errno));
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_nametoindex(can_interface_.c_str());

    // TX = command to node
    addr.can_addr.tp.tx_id = node_id;

    // RX = response from node (common +0x80 pattern, adjust if needed)
    addr.can_addr.tp.rx_id = node_id + 0x80;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(get_logger(), "ISO-TP bind failed: %s", strerror(errno));
        close(sock);
        return false;
    }

    iso_sockets_[node_id] = {sock};
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
  int16_t x = static_cast<int16_t>(msg->x * 10.0);
  int16_t y = static_cast<int16_t>(msg->y * 10.0);
  int16_t z = static_cast<int16_t>(msg->z * 10.0);
  int16_t speed = static_cast<int16_t>(msg->speed * 10.0);
  if (msg->command_type == hexapod_msgs::msg::LegCommand::LINEAR) {
    append_bytes(&x, sizeof(x));
    append_bytes(&y, sizeof(y));
    append_bytes(&z, sizeof(z));
    append_bytes(&speed, sizeof(speed));
  }
  else if (msg->command_type == hexapod_msgs::msg::LegCommand::RAPID) {
    append_bytes(&x, sizeof(x));
    append_bytes(&y, sizeof(y));
    append_bytes(&z, sizeof(z));
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

  std::lock_guard<std::mutex> lock(command_mutex_);

  for (auto node_id : target_nodes)
  {
      uint32_t leg =
          node_id - node_id_;

      if (leg < pending_commands_.size())
      {
          pending_commands_[leg].payload = payload;
          pending_commands_[leg].valid = true;
      }
  }
}

void CanInterface::scheduler_loop()
{
    using clock = std::chrono::steady_clock;

    auto next_tick = clock::now();

    while (scheduler_running_ && rclcpp::ok())
    {
        next_tick += std::chrono::milliseconds(100);

        std::array<PendingLegCommand, 6> commands;

        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            commands = pending_commands_;
            for (auto &cmd : pending_commands_)
            {
                cmd.valid = false;
                cmd.payload.clear();
            }
        }

        for (size_t leg = 0; leg < commands.size(); ++leg)
        {
            if (!commands[leg].valid)
                continue;

            uint32_t node_id =
                node_id_ +
                static_cast<uint32_t>(leg);

            send_isotp(
                node_id,
                commands[leg].payload);
        }

        std::this_thread::sleep_until(next_tick);
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
  bool success = write(sockfd_, &frame, sizeof(frame)) ==
         static_cast<ssize_t>(sizeof(frame));
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  return success;
}

// ===================== ISO-TP =====================
bool CanInterface::send_isotp(
    uint32_t node_id,
    const std::vector<uint8_t>& payload)
{
    auto it = iso_sockets_.find(node_id);
    if (it == iso_sockets_.end())
    {
        if (!create_isotp_socket(node_id))
            return false;
        it = iso_sockets_.find(node_id);
    }

    int sock = it->second.sockfd;

    auto start = std::chrono::steady_clock::now();

    ssize_t n = write(sock, payload.data(), payload.size());

    auto end = std::chrono::steady_clock::now();

    RCLCPP_INFO(
        get_logger(),
        "Leg %u write took %ld us",
        node_id,
        std::chrono::duration_cast<std::chrono::microseconds>(
            end - start).count());

      if (n < 0)
      {
          RCLCPP_ERROR(get_logger(),
              "ISO-TP write failed to 0x%X: %s",
              node_id, strerror(errno));
          return false;
      }
      if (n != static_cast<ssize_t>(payload.size()))
      {
          RCLCPP_ERROR(get_logger(),
              "ISO-TP write failed to 0x%X: incomplete write",
              node_id);
          return false;
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

void CanInterface::can_receive_loop()
{
    while (rclcpp::ok())
    {
        for (auto &kv : iso_sockets_)
        {
            int sock = kv.second.sockfd;

            uint8_t buffer[4096];

            ssize_t len = read(sock, buffer, sizeof(buffer));

            if (len > 0)
            {
                handle_isotp_message(kv.first, buffer, len);
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
}

void CanInterface::handle_isotp_message(
    uint32_t node_id,
    const uint8_t* data,
    size_t len)
{
    RCLCPP_INFO(get_logger(),
        "ISO-TP RX from 0x%X len=%zu",
        node_id, len);

    // decode directly into ROS messages
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