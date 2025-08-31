#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class TestListener : public rclcpp::Node
{
public:
    TestListener()
    : Node("test_listener")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable().transient_local();  // QoS profile

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "topic", qos, std::bind(&TestListener::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestListener>());
    rclcpp::shutdown();
    return 0;
}
