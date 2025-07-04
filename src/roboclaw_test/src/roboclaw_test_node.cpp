
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cstdint>

class RoboClawTestNode : public rclcpp::Node
{
public:
    RoboClawTestNode() : Node("roboclaw_test_node")
    {
        std::string port = "/dev/ttyACM0";  // 実際の接続先に応じて変更
        int baudrate = B115200;              // Motion Studioで設定したボーレートに合わせる

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

        // 実際に送信するコマンド
        send_forward_command(0x80, 64);  // Address 0x80、速度64で正転
    }

    ~RoboClawTestNode() {
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;

    uint16_t crc16(const std::vector<uint8_t>& buffer) {
        uint16_t crc = 0;
        for (auto byte : buffer) {
            crc ^= (static_cast<uint16_t>(byte) << 8);
            for (int i = 0; i < 8; i++) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc <<= 1;
            }
        }
        return crc;
    }

    void send_forward_command(uint8_t address, uint8_t speed) {
        std::vector<uint8_t> packet = {address, 0x00, speed};  // 0x00 = ForwardM1コマンド
        uint16_t crc = crc16(packet);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(crc & 0xFF);
        write(fd_, packet.data(), packet.size());
        RCLCPP_INFO(this->get_logger(), "Sent forward command (speed=%d)", speed);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboClawTestNode>());
    rclcpp::shutdown();
    return 0;
}
