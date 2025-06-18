#include "rclcpp/rclcpp.hpp"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "sensor_msgs/msg/joy.hpp"
#include <thread>

class DynamixelControllerJoy : public rclcpp::Node
{
public:
  DynamixelControllerJoy()
  : Node("dynamixel_controllerjoy"), dxl_id_(1)
  {
    const char *device_name = "/dev/ttyUSB0";
    const uint32_t baud_rate = 115200;

    if (!dxl_wb.init(device_name, baud_rate)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize Dynamixel Workbench");
      rclcpp::shutdown();
      return;
    }

    if (!dxl_wb.ping(dxl_id_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to ping Dynamixel ID %d", dxl_id_);
      rclcpp::shutdown();
      return;
    }

    if (!dxl_wb.torqueOff(dxl_id_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to torque off");
      rclcpp::shutdown();
      return;
    }

    if (!dxl_wb.itemWrite(dxl_id_, "Operating_Mode", 3)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set Operating Mode to Position Mode");
      rclcpp::shutdown();
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!dxl_wb.jointMode(dxl_id_, 0, 4095)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set joint mode for ID %d", dxl_id_);
      rclcpp::shutdown();
      return;
    }

    if (!dxl_wb.itemWrite(dxl_id_, "Profile_Acceleration", 20)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set Profile Acceleration");
      rclcpp::shutdown();
      return;
    }

    if (!dxl_wb.itemWrite(dxl_id_, "Profile_Velocity", 100)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set Profile Velocity");
      rclcpp::shutdown();
      return;
    }

    if (!dxl_wb.torqueOn(dxl_id_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to torque on");
      rclcpp::shutdown();
      return;
    }

    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      [this](sensor_msgs::msg::Joy::UniquePtr msg) {
        RCLCPP_INFO(this->get_logger(), "Axes:");
        for (size_t i = 0; i < msg->axes.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "  [%zu]: %f", i, msg->axes[i]);
          if (!dxl_wb.itemWrite(dxl_id_, "Goal_Position", static_cast<int32_t>(msg->axes[i] * 2048))) {
            RCLCPP_FATAL(this->get_logger(), "Failed to write Goal_Position");
            rclcpp::shutdown();
            return;
          }
        }
      });
  }

private:
  DynamixelWorkbench dxl_wb;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  uint8_t dxl_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DynamixelControllerJoy>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
