#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>
#include <string>

#include <linux/can.h>

#include "hexapod_msgs/msg/leg_command.hpp"

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

private:
  int sockfd_;
  std::string can_interface_;
  uint32_t node_id_;

  rclcpp::Subscription<hexapod_msgs::msg::LegCommand>::SharedPtr command_sub_;

  std::map<int, std::vector<uint32_t>> leg_groups_;
};