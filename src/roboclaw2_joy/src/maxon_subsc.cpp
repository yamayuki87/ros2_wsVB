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
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<float>("alpha", 0.2f);
        this->declare_parameter<float>("deadzone", 0.05f);
        

        this->get_parameter("port", port);
        this->get_parameter("baudrate", baudrate);
        this->get_parameter("alpha", alpha);
        this->get_parameter("deadzone", deadzone);

        // シリアルポート設定
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s", port.c_str());
            rclcpp::shutdown();
            return;
        }

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            close(fd_);  
            rclcpp::shutdown();
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
    
        if (tcsetattr(fd_,TCSANOW,&tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            close(fd_);  
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port opened");

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                if (msg->axes.size() < 5) return;



                /*control moter1*/
                float raw1 = msg->axes[1];
                if(std::abs(raw1)<deadzone){
                    raw1=0;
                }
                int target_speed1 = static_cast<int>(std::round(raw1 * 127));
                smoothed_speed1_ = static_cast<int>(
                    std::round(alpha * target_speed1 + (1 - alpha) * smoothed_speed1_)
                );
                if(std::abs(smoothed_speed1_)<=5)
                    smoothed_speed1_=0;

                if (smoothed_speed1_ >= 0) {
                    send_command(0x80, 0x00, static_cast<uint8_t>(smoothed_speed1_)); //moter1 nomal ratation
                } else{
                    send_command(0x80, 0x01, static_cast<uint8_t>(-smoothed_speed1_)); //moter1 reversal ratation
                }
            

                /*control moter2*/
                float raw2 = msg->axes[4];
                if(std::abs(raw2)<deadzone){
                    raw2=0;
                }
                int target_speed2 = static_cast<int>(std::round(raw2 * 127));
                smoothed_speed2_ = static_cast<int>(
                    std::round(alpha * target_speed2 + (1 - alpha) * smoothed_speed2_)
                );
                if(std::abs(smoothed_speed2_)<=5) 
                    smoothed_speed2_=0;

                if (smoothed_speed2_ >= 0) {
                    send_command(0x80, 0x04, static_cast<uint8_t>(smoothed_speed2_)); //moter2 nomal ratation
                } else{
                    send_command(0x80, 0x05, static_cast<uint8_t>(-smoothed_speed2_)); //moter2 reversal ratation
                }

            });
    }

    ~RoboClawJoyNode() {
        send_command(0x80, 0x00, static_cast<uint8_t>(0)); 
        send_command(0x80, 0x01, static_cast<uint8_t>(0)); 
        send_command(0x80, 0x04, static_cast<uint8_t>(0)); 
        send_command(0x80, 0x05, static_cast<uint8_t>(0));
        RCLCPP_INFO(this->get_logger(), "Shutting down, stopping motors.");
        if (fd_ >= 0) close(fd_);
    }

private:
    int fd_;
    int smoothed_speed1_ = 0;
    int smoothed_speed2_ = 0;
    std::string port;
    int baudrate;
    float alpha;
    float deadzone ;
    

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

    void send_command(uint8_t address, uint8_t command, uint8_t speed) {
        std::vector<uint8_t> packet = {address, command, speed};
        uint16_t crc = crc16(packet);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(crc & 0xFF);
        write(fd_, packet.data(), packet.size());
        
        RCLCPP_INFO(this->get_logger(), "Sent command: 0x%02X speed=%s%d", command, 
        (command==0x00 || command==0x04) ? "+":"-",speed);
        
        
        
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboClawJoyNode>());
    rclcpp::shutdown();
    return 0;
}
