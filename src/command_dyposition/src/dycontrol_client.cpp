#include "rclcpp/rclcpp.hpp"
#include "command_dyposition/srv/command_value.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: dynamixek_control_client X"); //set postion value
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("position_client");
  rclcpp::Client<command_dyposition::srv::CommandValue>::SharedPtr client =
    node->create_client<command_dyposition::srv::CommandValue>("dynamixel_control");

  auto request = std::make_shared<command_dyposition::srv::CommandValue::Request>();
  request->value = atoll(argv[1]);
  

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Succesed to set correct position");

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to set correct position");
  }

  rclcpp::shutdown();
  return 0;
}