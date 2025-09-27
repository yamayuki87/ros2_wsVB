#include "rclcpp/rclcpp.hpp"
#include "command_dyposition_sdk/srv/command_value.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

using std::placeholders::_1;
using std::placeholders::_2;

class TestDynamixelSDK : public rclcpp::Node
{
public:
    TestDynamixelSDK()
    : Node("test_dynamixel_sdk")
    {
        // Port and baudrate
        std::string port_name = "/dev/ttyUSB0";
        int baudrate = 57600;

        portHandler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!portHandler->openPort()) {
            throw std::runtime_error("Failed to open port");
        }

        if (!portHandler->setBaudRate(baudrate)) {
            throw std::runtime_error("Failed to set baudrate");
        }

        // サービスサーバー作成
        service_ = this->create_service<command_dyposition_sdk::srv::CommandValue>(
            "dynamixel_position",
            std::bind(&TestDynamixelSDK::position_control, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "Dynamixel Position Control Server started.");
    }

private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    rclcpp::Service<command_dyposition_sdk::srv::CommandValue>::SharedPtr service_;

    void position_control(
        std::shared_ptr<command_dyposition_sdk::srv::CommandValue::Request> request,
        std::shared_ptr<command_dyposition_sdk::srv::CommandValue::Response> response)
    {
        uint8_t dxl_id = 1;
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;

        // Goal Position 書き込み
        int32_t goal_position = request->value;

        RCLCPP_INFO(this->get_logger(),
            "Motor moving to target position: %ld", request->value);

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler,
            dxl_id,
            116,  // Goal Position アドレス（XM540）
            goal_position,
            &dxl_error
        );

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write Goal Position.");
            response->success = false;
            return;
        }

        // 位置監視ループ
        while (rclcpp::ok()) {
            int32_t present_position = 0;
            dxl_comm_result = packetHandler->read4ByteTxRx(
                portHandler,
                dxl_id,
                132,  // Present Position アドレス（XM540）
                (uint32_t*)&present_position,
                &dxl_error
            );

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Failed to read Present Position.");
                response->success = false;
                return;
            }

            if (abs(goal_position - present_position) < 20) {
                RCLCPP_INFO(this->get_logger(),
                    "Position reached: %d (target: %ld)",
                    present_position, request->value);
                response->success = true;
                return;
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Position error: current %d, target %ld",
                    present_position, request->value);
            }

            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestDynamixelSDK>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
