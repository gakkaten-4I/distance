#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <windows.h>
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
        if (h_serial_ != INVALID_HANDLE_VALUE) {
            CloseHandle(h_serial_);
        }
        if (reading_thread_.joinable()) {
            reading_thread_.join();
        }
    }

private:
    HANDLE h_serial_ = INVALID_HANDLE_VALUE;
    std::string serial_name_;
    int baudrate_;
    std::thread reading_thread_;
    std::mutex mtx_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_publisher_;
    std::chrono::seconds reconnect_interval_;

    bool open_serial_port(const std::string &port, int baudrate) {
        h_serial_ = CreateFile(port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

        if (h_serial_ == INVALID_HANDLE_VALUE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
            return false;
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(h_serial_, &dcbSerialParams)) {
            RCLCPP_ERROR(this->get_logger(), "Error from GetCommState");
            CloseHandle(h_serial_);
            return false;
        }

        dcbSerialParams.BaudRate = baudrate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(h_serial_, &dcbSerialParams)) {
            RCLCPP_ERROR(this->get_logger(), "Error from SetCommState");
            CloseHandle(h_serial_);
            return false;
        }

        // Set timeouts
        COMMTIMEOUTS timeouts = { 0 };
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        SetCommTimeouts(h_serial_, &timeouts);

        RCLCPP_INFO(this->get_logger(), "Successfully opened serial port %s", port.c_str());
        return true;
    }

    void close_serial_port() {
        if (h_serial_ != INVALID_HANDLE_VALUE) {
            CloseHandle(h_serial_);
            h_serial_ = INVALID_HANDLE_VALUE;
        }
    }

    void read_sensor_data() {
        RCLCPP_INFO(this->get_logger(), "Started reading sensor data from %s", serial_name_.c_str());

        while (rclcpp::ok()) {
            if (h_serial_ == INVALID_HANDLE_VALUE) {
                if (!open_serial_port(serial_name_, baudrate_)) {
                    std::this_thread::sleep_for(reconnect_interval_);
                    continue;
                }
            }

            std::vector<uint8_t> buff(TOF_LENGTH);
            DWORD bytes_read = 0;

            // Flush the serial port buffer before reading
            PurgeComm(h_serial_, PURGE_RXCLEAR | PURGE_TXCLEAR);

            while (bytes_read < TOF_LENGTH) {
                DWORD bytes_to_read = TOF_LENGTH - bytes_read;
                DWORD bytes_now = 0;
                if (!ReadFile(h_serial_, buff.data() + bytes_read, bytes_to_read, &bytes_now, NULL)) {
                    RCLCPP_ERROR(this->get_logger(), "Read error on port %s, attempting to reconnect...", serial_name_.c_str());
                    close_serial_port();
                    std::this_thread::sleep_for(reconnect_interval_);
                    break;
                }
                bytes_read += bytes_now;
            }

            if (bytes_read == TOF_LENGTH) {
                std::lock_guard<std::mutex> lock(mtx_);
                if (buff[0] == TOF_HEADER[0] && buff[1] == TOF_HEADER[1] && buff[2] == TOF_HEADER[2] && verify_checksum(buff, TOF_LENGTH)) {
                    uint16_t signal = (buff[12]) | (buff[13] << 8);
                    if (signal == 0) {
                        RCLCPP_INFO(this->get_logger(), "Sensor data: Out of range!");
                    } else {
                        uint32_t system_time = (buff[4]) | (buff[5] << 8) | (buff[6] << 16) | (buff[7] << 24);
                        uint32_t distance = (buff[8]) | (buff[9] << 8) | (buff[10] << 16);

                        auto msg = std_msgs::msg::Float64();
                        msg.data = static_cast<double>(distance);
                        distance_publisher_->publish(msg);

                        RCLCPP_INFO(this->get_logger(), "Sensor data: id=%d, system_time=%u, distance=%u, status=%d, signal=%d",
                                    buff[3], system_time, distance, buff[11], signal);
                    }
                } else {
                    std::ostringstream oss;
                    oss << "Invalid or corrupted data received: ";
                    for (const auto &byte : buff) {
                        oss << "0x" << std::hex << static_cast<int>(byte) << " ";
                    }
                    RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Incomplete data received: %lu bytes", bytes_read);
            }
        }
    }

    bool verify_checksum(const std::vector<uint8_t> &data, size_t length) {
        if (length > data.size()) {
            std::cerr << "Data length is shorter than expected." << std::endl;
            return false;
        }

        uint32_t checksum = 0;
        for (size_t i = 0; i < length - 1; ++i) {
            checksum += data[i];
        }
        checksum = checksum % 256;

        if (checksum == data[length - 1]) {
            std::cout << "TOF data is ok!" << std::endl;
            return true;
        } else {
            std::cerr << "TOF data is error!" << std::endl;
            return false;
        }
    }

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

    std::vector<std::string> serial_ports = {"COM7", "COM8"}; // Windowsのポート指定

    std::vector<std::shared_ptr<SensorNode>> nodes;
    for (const auto &port : serial_ports) {
        nodes.push_back(std::make_shared<SensorNode>(port, CBR_115200));
    }

    auto dummy_node = rclcpp::Node::make_shared("dummy_node");
    rclcpp::spin(dummy_node);
    rclcpp::shutdown();
    return 0;
}
