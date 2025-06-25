#include "rclcpp/rclcpp.hpp"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "command_dyposition/srv/command_value.hpp"
#include <thread>
#include <chrono>
#include <stdexcept>
#include <string>
#include <functional>

class TestDynamixel : public rclcpp::Node {
public:
    TestDynamixel()
    : Node("position_server"),
      port_name("/dev/ttyUSB0"),
      baud_rate(115200),
      dxl_id(1),
      result(false),
      model_number(0)
    {
        // Initialize Dynamixel Workbench
        result = dxl_wb.init(port_name.c_str(), baud_rate, nullptr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init Dynamixel");
            throw std::runtime_error("Failed to init Dynamixel");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to init (baud: %d)", baud_rate);

        // Ping test
        const char* log_cstr = nullptr;
        result = dxl_wb.ping(dxl_id, &model_number, &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            throw std::runtime_error("Failed to ping Dynamixel");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to ping");
        RCLCPP_INFO(this->get_logger(), "id: %d, model_number: %d", dxl_id, model_number);

        // Change to Joint Mode
        log_cstr = nullptr;
        result = dxl_wb.jointMode(dxl_id, 0, 0, &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            throw std::runtime_error("Failed to change joint mode");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to change joint mode");

        // Get present position
        int32_t position = 0;
        log_cstr = nullptr;
        result = dxl_wb.getPresentPositionData(dxl_id, &position, &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            throw std::runtime_error("Failed to get present position data");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to get present position data");



        service_=this->create_service<command_dyposition::srv::CommandValue>(
        "dynamixel_control",
        std::bind(&TestDynamixel::position_control,this,
            std::placeholders::_1,std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(),"Service ready");
    }
       

private:
    void position_control(
        const std::shared_ptr<command_dyposition::srv::CommandValue::Request> request,
        std::shared_ptr<command_dyposition::srv::CommandValue::Response> response)
    {
        

        const char* log_cstr = nullptr;
        log_cstr = nullptr;
        result = dxl_wb.goalPosition(dxl_id, static_cast<int32_t>(request->value), &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            throw std::runtime_error("Failed to operate the motor");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to operate the motor");

        std::this_thread::sleep_for(std::chrono::seconds(3));
        RCLCPP_INFO(this->get_logger(), "Dynamixel is moving...");
        
        const int32_t threshold = 50;  // 許容誤差（例）
        int32_t current_pos = 0;
        if (dxl_wb.getPresentPositionData(dxl_id, &current_pos, &log_cstr)) {
            response->success = (std::abs(current_pos - request->value) < threshold);
        } else {
            response->success = false;
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
        }

        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Succeeded to operate the motor");
        }else{
            RCLCPP_INFO(this->get_logger(), "failed to operate the motor");
        }


    }

    DynamixelWorkbench dxl_wb;
    const std::string port_name;
    const int baud_rate;
    const uint8_t dxl_id;
    bool result;
    uint16_t model_number;

    rclcpp::Service<command_dyposition::srv::CommandValue>::SharedPtr service_;
};

// Set up ROS2 node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<TestDynamixel>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}