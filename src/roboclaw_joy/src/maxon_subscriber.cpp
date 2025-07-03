#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <algorithm>  

class RoboClawSpeedNode : public rclcpp::Node
{
public:
    RoboClawSpeedNode() : Node("roboclaw_speed_node")
    {
        std::string port = "/dev/ttyACM0";  // 実際のポートに合わせる
        int baudrate = B115200;

        // シリアルポートオープン
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            return;
        }
        cfsetospeed(&tty, baudrate);
        cfsetispeed(&tty, baudrate);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tcsetattr(fd_, TCSANOW, &tty);

        RCLCPP_INFO(this->get_logger(), "Serial port opened");

        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](sensor_msgs::msg::Joy::UniquePtr msg) {
                if (msg->axes.size() > 0) {
                    float input = msg->axes[0];
                    input = std::clamp(input, -1.0f, 1.0f);  // 安全対策

                    int32_t speed = static_cast<int32_t>(input * max_speed_);
                    RCLCPP_INFO(this->get_logger(), "Axes[0]: %f -> Speed: %d", input, speed);
                    sendSetM1Speed(0x80, speed);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Joy axes[0] not available");
                }
                
    });

    }

    ~RoboClawSpeedNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;
    const int32_t max_speed_ = 50000;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    uint16_t crc16(const std::vector<uint8_t>& buffer) {
        uint16_t crc = 0;
        for (auto byte : buffer) {
            crc ^= static_cast<uint16_t>(byte) << 8;
            for (int i = 0; i < 8; i++) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc <<= 1;
            }
        }
        return crc;
    }

    void sendSetM1Speed(uint8_t address, int32_t speed) {
        std::vector<uint8_t> packet;
        packet.push_back(address);          // Address (例: 0x80)
        packet.push_back(35);               // Command: SetM1Speed (35)

        // Speedはint32_t、LSBファーストで送信
        packet.push_back((speed >> 24) & 0xFF);
        packet.push_back((speed >> 16) & 0xFF);
        packet.push_back((speed >> 8) & 0xFF);
        packet.push_back(speed & 0xFF);

        uint16_t crc = crc16(packet);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(crc & 0xFF);

        write(fd_, packet.data(), packet.size());

        RCLCPP_INFO(this->get_logger(), "Sent SetM1Speed: %d", speed);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboClawSpeedNode>());
    rclcpp::shutdown();
    return 0;
}
