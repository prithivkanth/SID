#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include <chrono>


using namespace std::chrono_literals;


class SerialReciever : public rclcpp:: Node
{
public:
    SerialReciever() : Node("simple_serial_reciever")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = get_parameter("port").as_string();

        esp32_.Open(port_);
        esp32_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        pub_ = create_publisher<std_msgs::msg::String>("serial_reciever", 10);

        timer_ = create_wall_timer(0.01s, std::bind(&SerialReciever::timerCallback, this));
    }

    ~SerialReciever()
    {
        esp32_.Close();
    }


    void timerCallback()
    {
        if(rclcpp::ok() && esp32_.IsDataAvailable())
        {
            auto message = std_msgs::msg::String();
            esp32_.ReadLine(message.data);
            pub_->publish(message);
        }
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    std::string port_;
    LibSerial::SerialPort esp32_;
    rclcpp::TimerBase::SharedPtr timer_;
};