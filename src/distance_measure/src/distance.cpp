#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <sstream>
#include "std_msgs/msg/float64.hpp"

#define TOF_LENGTH 16
const uint8_t TOF_HEADER[3] = {87, 0, 255};

using namespace std::chrono_literals;

class SensorNode : public rclcpp::Node {
public:
    SensorNode(const std::string &serial_name, int baudrate)
        : Node(generate_node_name(serial_name)), serial_name_(serial_name), baudrate_(baudrate), reconnect_interval_(3s) {

        this->declare_parameter<std::string>("serial_name", serial_name);
        this->declare_parameter<int>("baudrate", baudrate);

        // Create a publisher for the distance data
        distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>(generate_topic_name("distance"), 10);

        // Create a thread for reading sensor data
        reading_thread_ = std::thread(&SensorNode::read_sensor_data, this);
    }

    ~SensorNode() {
        if (fd_ >= 0) {
            close(fd_);
        }
        if (reading_thread_.joinable()) {
            reading_thread_.join();
        }
    }

private:
    int fd_ = -1;
    std::string serial_name_;
    int baudrate_;
    std::thread reading_thread_;
    std::mutex mtx_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;
    std::chrono::seconds reconnect_interval_;

    bool open_serial_port(const std::string &port, int baudrate) {
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            close(fd_);
            return false;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;  // Non-blocking
        tty.c_cc[VTIME] = 0; // No timeout

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
            close(fd_);
            return false;
        }

        // Flush the serial port buffers
        tcflush(fd_, TCIOFLUSH);
        RCLCPP_INFO(this->get_logger(), "Successfully opened serial port %s", port.c_str());

        return true;
    }

    void close_serial_port() {
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

    void read_sensor_data() {
        RCLCPP_INFO(this->get_logger(), "Started reading sensor data from %s", serial_name_.c_str());

        while (rclcpp::ok()) {
            // Check if the serial port is open, otherwise attempt to reconnect
            if (fd_ < 0) {
                if (!open_serial_port(serial_name_, baudrate_)) {
                    std::this_thread::sleep_for(reconnect_interval_);
                    continue; // Retry after waiting for the interval
                }
            }

            std::vector<uint8_t> buff(TOF_LENGTH);
            size_t bytes_received = 0;

            // Flush the serial port buffer before reading
            tcflush(fd_, TCIFLUSH);

            // Read data until we have enough bytes
            while (bytes_received < TOF_LENGTH) {
                ssize_t n = read(fd_, buff.data() + bytes_received, TOF_LENGTH - bytes_received);
                if (n > 0) {
                    bytes_received += n;
                } else if (n < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Read error on port %s, attempting to reconnect...", serial_name_.c_str());
                    close_serial_port();
                    std::this_thread::sleep_for(reconnect_interval_);
                    break; // Exit the inner loop and attempt to reconnect
                }
            }

            if (bytes_received == TOF_LENGTH) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (buff[0] == TOF_HEADER[0] && buff[1] == TOF_HEADER[1] && buff[2] == TOF_HEADER[2] && verify_checksum(buff, TOF_LENGTH)) {
                    uint16_t signal = (buff[12]) | (buff[13] << 8);
                    if (signal == 0) {
                        RCLCPP_INFO(this->get_logger(), "Sensor data: Out of range!");
                    } else {
                        uint32_t system_time = (buff[4]) | (buff[5] << 8) | (buff[6] << 16) | (buff[7] << 24);
                        uint32_t distance = (buff[8]) | (buff[9] << 8) | (buff[10] << 16);

                        // Publish the distance data
                        auto msg = std_msgs::msg::Float64();
                        msg.data = static_cast<double>(distance);
                        distance_publisher_->publish(msg);

                        RCLCPP_INFO(this->get_logger(), "verify_checksum: %d", verify_checksum(buff, TOF_LENGTH));

                        RCLCPP_INFO(this->get_logger(), "Sensor data: id=%d, system_time=%u, distance=%u, status=%d, signal=%d",
                                    buff[3], system_time, distance, buff[11], signal);
                    }
                } else {
                    std::ostringstream oss;
                    RCLCPP_INFO(this->get_logger(), "verify_checksum: %d", verify_checksum(buff, TOF_LENGTH));
                    oss << "Invalid or corrupted data received: ";
                    for (const auto &byte : buff) {
                        oss << "0x" << std::hex << static_cast<int>(byte) << " ";
                    }
                    RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Incomplete data received: %zu bytes", bytes_received);
            }
        }
    }

    bool verify_checksum(const std::vector<uint8_t> &data, size_t length) {
        if (length > data.size()) {
            std::cerr << "Data length is shorter than expected." << std::endl;
            return false;
        }

        uint32_t checksum = 0;
        // チェックサムを計算
        for (size_t i = 0; i < length - 1; ++i) {
            checksum += data[i];
        }
        checksum = checksum % 256;

        // チェックサムが一致するか確認
        if (checksum == data[length - 1]) {
            std::cout << "TOF data is ok!" << std::endl;
            return true;
        } else {
            std::cerr << "TOF data is error!" << std::endl;
            return false;
        }
    }

    // Helper function to generate a valid node name
    static std::string generate_node_name(const std::string &serial_name) {
        std::string base_name = "sensor_node";
        std::string sanitized_name;
        for (char c : serial_name) {
            if (c == '/' || c == '-') {
                sanitized_name += '_';
            } else {
                sanitized_name += c;
            }
        }
        return base_name + "_" + sanitized_name;
    }

    // Helper function to generate a valid topic name
    std::string generate_topic_name(const std::string &base_name) {
        std::string sanitized_name;
        for (char c : serial_name_) {
            if (c == '/' || c == '-') {
                sanitized_name += '_';
            } else {
                sanitized_name += c;
            }
        }
        return base_name + "_" + sanitized_name;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // List of serial ports
    // std::vector<std::string> serial_ports = {"/dev/tof0", "/dev/tof1", "/dev/tof2", "/dev/tof3", "/dev/tof4", "/dev/tof5", "/dev/tof6", "/dev/tof7"}; // Example ports
    std::vector<std::string> serial_ports = {"/dev/ttyUSB0"};

    // Vector to hold sensor nodes
    std::vector<std::shared_ptr<SensorNode>> nodes;
    for (const auto &port : serial_ports) {
        nodes.push_back(std::make_shared<SensorNode>(port, 115200));
    }

    // Spin with a dummy node to keep the process alive
    auto dummy_node = rclcpp::Node::make_shared("dummy_node");
    rclcpp::spin(dummy_node);
    rclcpp::shutdown();
    return 0;
}