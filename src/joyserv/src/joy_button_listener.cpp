#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyButtonListener : public rclcpp::Node
{
public:
  JoyButtonListener()
  : Node("joy_button_listener")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyButtonListener::joy_callback, this, std::placeholders::_1));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons.size() > 0 && msg->buttons[0] == 1) {
      RCLCPP_INFO(this->get_logger(), "Button A pressed!");
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyButtonListener>());
  rclcpp::shutdown();
  return 0;
}
