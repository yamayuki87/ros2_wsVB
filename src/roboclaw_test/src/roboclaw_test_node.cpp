// src/roboclaw_joy_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cmath>
#include <limits>

class RoboClawJoyNode : public rclcpp::Node
{
public:
    RoboClawJoyNode() : Node("roboclaw_joy_node"), fd_(-1), last_speed_(std::numeric_limits<int>::min())
    {
        const std::string port = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        const int baudrate_param = this->declare_parameter<int>("baudrate", 115200);

        const speed_t baudrate = get_baudrate(baudrate_param);

        if (!open_serial_port(port, baudrate)) {
            rclcpp::shutdown();
            return;
        }

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                if (msg->axes.empty()) return;

                float raw = msg->axes[0];
                int speed = static_cast<int>(std::round(raw * 127));

                if (speed >= 0) {
                    send_command(0x80, 0x00, static_cast<uint8_t>(speed));
                } else {
                    send_command(0x80, 0x01, static_cast<uint8_t>(-speed));
                }
            });
    }

    ~RoboClawJoyNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;
    int last_speed_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    speed_t get_baudrate(int baudrate_param) {
        switch (baudrate_param) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default:
                RCLCPP_WARN(this->get_logger(), "Unsupported baudrate %d, defaulting to 115200", baudrate_param);
                return B115200;
        }
    }

    bool open_serial_port(const std::string &port, speed_t baudrate) {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s", port.c_str());
            return false;
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
        return true;
    }

    uint16_t crc16(const std::vector<uint8_t>& buffer) {
        uint16_t crc = 0;
        for (auto byte : buffer) {
            crc ^= static_cast<uint16_t>(byte) << 8;
            for (int i = 0; i < 8; i++) {
                crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
            }
        }
        return crc;
    }

    void send_command(uint8_t address, uint8_t command, uint8_t speed) {
        std::vector<uint8_t> packet = {address, command, speed};
        uint16_t crc = crc16(packet);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(crc & 0xFF);

        write(fd_, packet.data(), packet.size());

        if (last_speed_ != static_cast<int>(speed)) {
            RCLCPP_INFO(this->get_logger(), "Sent command (cmd=%d, speed=%d)", command, speed);
            last_speed_ = speed;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboClawJoyNode>());
    rclcpp::shutdown();
    return 0;
}
