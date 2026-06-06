#include <rclcpp/rclcpp.hpp>
#include "can/msg/leg_command.hpp"
#include <chrono>

class LegCommandPublisher : public rclcpp::Node
{
public:
  LegCommandPublisher()
  : Node("leg_command_publisher")
  {
    publisher_ = this->create_publisher<can::msg::LegCommand>("/leg_command", 10);

    // Timer to publish commands every 2 seconds
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&LegCommandPublisher::publish_commands, this));

    command_count_ = 0;
    RCLCPP_INFO(this->get_logger(), "LegCommandPublisher started. Publishing various command types...");
  }

private:
  void publish_commands()
  {
    auto msg = can::msg::LegCommand();

    switch (command_count_ % 3) {
      case 0: {
        // LINEAR move - Leg 0
        RCLCPP_INFO(this->get_logger(), "Publishing LINEAR move to leg 0");
        msg.command_type = can::msg::LegCommand::LINEAR;
        msg.leg_number = 0;
        msg.axis = 0;
        msg.x = 100;
        msg.y = 0;
        msg.z = -200;
        msg.speed = 150;
        msg.position = 0.15;
        break;
      }
      case 1: {
        // RAPID move - Leg 1
        RCLCPP_INFO(this->get_logger(), "Publishing RAPID move to leg 1");
        msg.command_type = can::msg::LegCommand::RAPID;
        msg.leg_number = 1;
        msg.axis = 0;
        msg.x = 100;
        msg.y = 0;
        msg.z = -200;
        msg.speed = 0.0;
        msg.position = 0.0;
        break;
      }
      case 2: {
        // SINGLE_AXIS move - Leg 2, axis 1 (Y-axis)
        RCLCPP_INFO(this->get_logger(), "Publishing SINGLE_AXIS move to leg 2, axis 1");
        msg.command_type = can::msg::LegCommand::SINGLE_AXIS;
        msg.leg_number = 2;
        msg.axis = 1;
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0f;
        msg.speed = 0.0;
        msg.position = 0.15;
        break;
      }
    }

    publisher_->publish(msg);
    command_count_++;
  }

  rclcpp::Publisher<can::msg::LegCommand>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int command_count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LegCommandPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
