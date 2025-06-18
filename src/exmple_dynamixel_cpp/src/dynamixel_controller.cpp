#include "rclcpp/rclcpp.hpp"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include <thread>

class DynamixelController : public rclcpp::Node
{
public:
    DynamixelController() : Node("dynamixel_controller")
    {
        const char *device_name = "/dev/ttyUSB0";
        const uint32_t baud_rate = 115200;
        const uint8_t dxl_id = 1;

        // Dynamixel初期化
        if (!dxl_wb.init(device_name, baud_rate))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize Dynamixel Workbench");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Dynamixel Workbench initialized successfully");

        // Ping確認
        if (!dxl_wb.ping(dxl_id))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to ping Dynamixel ID %d", dxl_id);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Dynamixel ID %d ping successful", dxl_id);

        // 現在の状態を確認
        int32_t current_mode = 0;
        if (dxl_wb.itemRead(dxl_id, "Operating_Mode", &current_mode))
        {
            RCLCPP_INFO(this->get_logger(), "Current Operating Mode: %d", current_mode);
        }

        // トルクOFF
        if (!dxl_wb.torqueOff(dxl_id))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to torque off");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Torque OFF successful");

        // 少し待機
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Position制限を確認・設定
        int32_t min_pos = 0, max_pos = 0;
        if (dxl_wb.itemRead(dxl_id, "Min_Position_Limit", &min_pos) &&
            dxl_wb.itemRead(dxl_id, "Max_Position_Limit", &max_pos))
        {
            RCLCPP_INFO(this->get_logger(), "Position limits - Min: %d, Max: %d", min_pos, max_pos);
        }

        // Position制限を設定（XM540-W270の場合）
        if (!dxl_wb.itemWrite(dxl_id, "Min_Position_Limit", 0))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Min_Position_Limit");
        }
        if (!dxl_wb.itemWrite(dxl_id, "Max_Position_Limit", 4095))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Max_Position_Limit");
        }

        // Operating Modeを設定（Position Control Mode）
        if (!dxl_wb.itemWrite(dxl_id, "Operating_Mode", 3))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to set Operating Mode to Position Mode");
            rclcpp::shutdown();  
            return;
        }
        int32_t op_mode = -1;
        if (dxl_wb.itemRead(dxl_id, "Operating_Mode", &op_mode)) {
          RCLCPP_INFO(this->get_logger(), "After write, Operating Mode: %d", op_mode);
        }


        // 待機
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Profile設定
        if (!dxl_wb.itemWrite(dxl_id, "Profile_Acceleration", 20))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Profile Acceleration");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Profile Acceleration set to 20");
        }

        if (!dxl_wb.itemWrite(dxl_id, "Profile_Velocity", 100))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Profile Velocity");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Profile Velocity set to 100");
        }

        // 待機
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // トルクON
        if (!dxl_wb.torqueOn(dxl_id))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to torque on");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Torque ON successful");

        // 少し待機してからPosition設定
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 現在のPosition確認
        int32_t current_pos = 0;
        if (dxl_wb.itemRead(dxl_id, "Present_Position", &current_pos))
        {
            RCLCPP_INFO(this->get_logger(), "Current Position before move: %d", current_pos);
        }

        // Goal Position設定（中央位置）
        int32_t target_position = 2048;
        if (!dxl_wb.itemWrite(dxl_id, "Goal_Position", target_position))
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to write Goal_Position");
            
            // Hardware Error Status確認
            int32_t hw_error = 0;
            if (dxl_wb.itemRead(dxl_id, "Hardware_Error_Status", &hw_error))
            {
                RCLCPP_ERROR(this->get_logger(), "Hardware Error Status: %d", hw_error);
            }
            
            // Present Position確認
            int32_t present_pos = 0;
            if (dxl_wb.itemRead(dxl_id, "Present_Position", &present_pos))
            {
                RCLCPP_ERROR(this->get_logger(), "Present Position: %d", present_pos);
            }
            
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Goal Position set to %d", target_position);

        // 移動完了まで待機
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 最終位置確認
        int32_t final_pos = 0;
        if (dxl_wb.itemRead(dxl_id, "Present_Position", &final_pos))
        {
            RCLCPP_INFO(this->get_logger(), "Final Position: %d", final_pos);
        }

        // Moving状態確認
        int32_t moving = 0;
        if (dxl_wb.itemRead(dxl_id, "Moving", &moving))
        {
            RCLCPP_INFO(this->get_logger(), "Moving status: %d", moving);
        }

        RCLCPP_INFO(this->get_logger(), "Dynamixel control completed successfully");
    }

private:
    DynamixelWorkbench dxl_wb;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelController>();
    
    // ノードを少し実行してから終了
    rclcpp::spin_some(node);
    
    rclcpp::shutdown();
    return 0;
}