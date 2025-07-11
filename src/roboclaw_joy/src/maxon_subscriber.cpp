
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cmath>

class RoboClawJoyNode : public rclcpp::Node
{
public:
    RoboClawJoyNode() : Node("roboclaw_joy_node")
    {
        std::string port = "/dev/ttyACM0";
        int baudrate = B115200;

        // シリアルポート設定
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        struct termios tty{};
        tcgetattr(fd_, &tty);
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

       
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                if (msg->axes.empty()) return;

                float raw = msg->axes[1];  // -1.0 ~ 1.0
                int target_speed = static_cast<int>(std::round(raw * 127));

                const float alpha = 0.2f;  // smoothing 係数
                smoothed_speed_ = static_cast<int>(
                std::round(alpha * target_speed + (1 - alpha) * smoothed_speed_)
                );

                if (smoothed_speed_ >= 0) {
                    send_forward_command(0x80, static_cast<uint8_t>(smoothed_speed_));
                } else {
                    send_backward_command(0x80, static_cast<uint8_t>(-smoothed_speed_));
                }
            });
    }

    ~RoboClawJoyNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;
    int smoothed_speed_ = 0;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

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

    void send_forward_command(uint8_t address, uint8_t speed) {
        std::vector<uint8_t> packet = {address, 0x00, speed};
        uint16_t crc = crc16(packet);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(crc & 0xFF);
        write(fd_, packet.data(), packet.size());
        RCLCPP_INFO(this->get_logger(), "Sent forward command (speed=%d)", speed);
    }

    void send_backward_command(uint8_t address, uint8_t speed) {
        std::vector<uint8_t> packet = {address, 0x01, speed};
        uint16_t crc = crc16(packet);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(crc & 0xFF);
        write(fd_, packet.data(), packet.size());
        RCLCPP_INFO(this->get_logger(), "Sent backward command (speed=%d)", speed);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboClawJoyNode>());
    rclcpp::shutdown();
    return 0;
}
