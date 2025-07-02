#include "rclcpp/rclcpp.hpp"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "command_dyposition/srv/command_value.hpp"
#include <thread>
#include <chrono>
#include <stdexcept>
#include <string>
#include <functional>
#include <cmath>  // abs()

class TestDynamixel : public rclcpp::Node {
public:
    TestDynamixel()
    : Node("position_server"),
      result(false),
      model_number(0)
    {
        // パラメータ宣言
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("dxl_id", 1);

        // パラメータ取得
        this->get_parameter("port_name", port_name);
        this->get_parameter("baud_rate", baud_rate);
        this->get_parameter("dxl_id", dxl_id);

        // Dynamixel初期化
        result = dxl_wb.init(port_name.c_str(), baud_rate, nullptr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init Dynamixel");
            throw std::runtime_error("Failed to init Dynamixel");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to init (baud: %d)", baud_rate);

        // Pingテスト
        const char* log_cstr = nullptr;
        result = dxl_wb.ping(dxl_id, &model_number, &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            throw std::runtime_error("Failed to ping Dynamixel");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to ping id: %d, model_number: %d", dxl_id, model_number);

        // Jointモードへ変更
        result = dxl_wb.jointMode(dxl_id, 0, 0, &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            throw std::runtime_error("Failed to change joint mode");
        }
        RCLCPP_INFO(this->get_logger(), "Succeeded to change joint mode");

        // 現在位置確認
        int32_t position = 0;
        result = dxl_wb.getPresentPositionData(dxl_id, &position, &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            throw std::runtime_error("Failed to get present position data");
        }
        RCLCPP_INFO(this->get_logger(), "Present position: %d", position);

        // サービス作成
        service_ = this->create_service<command_dyposition::srv::CommandValue>(
            "dynamixel_control",
            std::bind(&TestDynamixel::position_control, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Service 'dynamixel_control' ready");
    }

private:
    void position_control(
        const std::shared_ptr<command_dyposition::srv::CommandValue::Request> request,
        std::shared_ptr<command_dyposition::srv::CommandValue::Response> response)
    {
        const char* log_cstr = nullptr;

        // ゴール位置指令
        result = dxl_wb.goalPosition(dxl_id, static_cast<int32_t>(request->value), &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            response->success = false;
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Motor moving to target position: %d", request->value);

        // 動作待ち（簡易版：sleepで3秒待機）
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 現在位置再取得
        int32_t current_pos = 0;
        result = dxl_wb.getPresentPositionData(dxl_id, &current_pos, &log_cstr);
        if (!result) {
            RCLCPP_ERROR(this->get_logger(), "%s", log_cstr);
            response->success = false;
            return;
        }

        // 許容誤差内なら成功とみなす
        const int32_t threshold = 50;
        if (std::abs(current_pos - request->value) < threshold) {
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Position reached: %d (target: %d)", current_pos, request->value);
        } else {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "Position error: current %d, target %d", current_pos, request->value);
        }
    }

    // メンバ変数
    DynamixelWorkbench dxl_wb;
    bool result;
    uint16_t model_number;

    std::string port_name;
    int baud_rate;
    int dxl_id;

    rclcpp::Service<command_dyposition::srv::CommandValue>::SharedPtr service_;
};

// main関数
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
