#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

#include <chrono>

class SimpleSerialReceiver : public rclcpp::Node
{
public:
    SimpleSerialReceiver() : Node("simple_serial_publisher_cpp")
    {
        declare_parameter("port", "/dev/ttyACM0");
        port_ = this->get_parameter("port").as_string();

        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        pub_ = create_publisher<std_msgs::msg::String>("serial_communication", 10);

        //  Remember to add the pointer handler  &
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&SimpleSerialReceiver::timerCallback, this));

        RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    }

    ~SimpleSerialReceiver() // DESTRUCTOR
    {
        arduino_.Close();
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    LibSerial::SerialPort arduino_;
    std::string port_;


    void timerCallback()
    {   
        std_msgs::msg::String msg;

        if (rclcpp::ok() && arduino_.IsDataAvailable())
        {
            arduino_.ReadLine(msg.data);
        }

        pub_->publish(msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSerialReceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 

    return 0;
}