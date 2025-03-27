#include <chrono>
#include <memory>
#include <string>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <boost/asio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace boost::asio;
using namespace std::chrono_literals;

class GpchdGpsListener : public rclcpp::Node {
public:
    GpchdGpsListener() : Node("gpchd_gps_listener_node") {

        // 参数声明
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<int>("character_size", 8); //  声明 character_size


        // 获取参数值
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        int character_size = this->get_parameter("character_size").as_int(); // 获取 character_size
   
        try {
            io_service_ = std::make_shared<boost::asio::io_service>();
            serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_, serial_port);
            serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
            serial_port_->set_option(boost::asio::serial_port_base::character_size(character_size));
            serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            gps_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/local_messages_b", 10);
            speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/local_speed_b", 10);
            UI_llahs_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/ui_datapublisher_llahs", 10);

            strand_ = std::make_shared<boost::asio::strand<boost::asio::io_service::executor_type>>(io_service_->get_executor());
            read_serial_data();

            // 在独立线程中运行 io_service
            io_service_thread_ = std::thread([this]() { io_service_->run(); });


        } catch (const boost::system::system_error& ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", ex.what());
            exit(1);
        }
    }

    ~GpchdGpsListener() {
        io_service_->stop();
        if (io_service_thread_.joinable()) {
            io_service_thread_.join();
        }
    }


private:
    void read_serial_data() {
        buffer_.consume(buffer_.size()); // 清空缓冲区
        boost::asio::async_read_until(*serial_port_, buffer_, '\n', // 使用逗号作为分隔符
            boost::asio::bind_executor(*strand_, std::bind(&GpchdGpsListener::handle_read, this,
            std::placeholders::_1, std::placeholders::_2)));
    }
    void handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
        if (error) {
            if (error != boost::asio::error::eof) {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", error.message().c_str());
            }
            return;
        }

        std::istream is(&buffer_);
        std::string line;
        std::getline(is, line);

        RCLCPP_INFO(this->get_logger(), "Received raw data: %s", line.c_str()); // 打印原始数据


        double latitude, longitude, heading, speed;
        char comma;
        std::stringstream ss(line);

        if (ss >>  longitude >> comma >> latitude >> comma >> heading >> comma >> speed) {
            auto msg = std::make_shared<gps_msgs::msg::GPSFix>();
            msg->latitude = latitude;
            msg->longitude = longitude;
            msg->track = heading;

            RCLCPP_INFO(this->get_logger(), "Received GPS data: Latitude: %f, Longitude: %f, Heading: %f, Speed: %f",
                        latitude, longitude, heading, speed);

            publish_gps_data(msg, speed);
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to parse GPS data: '%s'", line.c_str());
        }

        read_serial_data();
    }

    void publish_gps_data(const gps_msgs::msg::GPSFix::SharedPtr msg, double speed) {
        gps_publisher_->publish(*msg);
        UI_llahs_publisher_->publish(*msg);

        auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = speed;
        speed_publisher_->publish(*twist_msg);
    }


    std::shared_ptr<boost::asio::io_service> io_service_;
    std::shared_ptr<boost::asio::serial_port> serial_port_;
    std::shared_ptr<boost::asio::strand<boost::asio::io_service::executor_type>> strand_;
    boost::asio::streambuf buffer_;
    std::thread io_service_thread_;

    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr UI_llahs_publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpchdGpsListener>());
    rclcpp::shutdown();
    return 0;
}
