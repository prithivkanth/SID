#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>


using std::placeholders::_1;

class SimpleTransmitter : public rclcpp::Node
{
public:
    SimpleTransmitter() : Node("simple_serial_transmitter")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = get_parameter("port").as_string();
        esp32_ .Open(port_);
        esp32_ .SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        sub_ = create_subscription<std_msgs::msg::String>("serial_transmitter", 10, std::bind(&SimpleTransmitter::msgCallback, this, _1));
    }

    ~SimpleTransmitter()
    {
        esp32_.Close();
    }


private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    LibSerial::SerialPort esp32_;
    std::string port_;

    void msgCallback(const std_msgs::msg::String &msg) 
    {
        esp32_.Write(msg.data);
    }
};

int main (int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTransmitter>());
    rclcpp::shutdown();
    return 0;
}