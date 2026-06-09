#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>
#include <string>
#include <array>
#include <mutex>

#include <linux/can.h>
#include "hexapod_msgs/msg/leg_command.hpp"

struct IsoTpSocket
{
    int sockfd;
};

struct PendingLegCommand
{
    bool valid = false;
    std::vector<uint8_t> payload;
};

class CanInterface : public rclcpp::Node
{
public:
  CanInterface();
  ~CanInterface() override;

private:
  bool init_can_socket();

  void on_command_received(const hexapod_msgs::msg::LegCommand::SharedPtr msg);

  bool send_can_frame(uint32_t can_id, const uint8_t* data, uint8_t dlc);

  bool send_isotp(uint32_t node_id, const std::vector<uint8_t>& payload);

  void load_leg_groups(const std::string& config_file);

  std::array<PendingLegCommand, 6> pending_commands_;
  std::mutex command_mutex_;

  std::thread scheduler_thread_;
  std::atomic<bool> scheduler_running_{false};

  void scheduler_loop();

  class CanInterface : public rclcpp::Node

  boolcreate_isotp_socket(uint32_t node_id)
  int sockfd_;
  std::string can_interface_;
  uint32_t node_id_;

  rclcpp::Subscription<hexapod_msgs::msg::LegCommand>::SharedPtr command_sub_;

  std::map<int, std::vector<uint32_t>> leg_groups_;
};