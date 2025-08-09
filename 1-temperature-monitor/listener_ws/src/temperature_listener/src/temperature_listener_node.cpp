#include <rclcpp/rclcpp.hpp>
#include "custom_temperature_msgs/msg/temperature_reading.hpp"
#include <deque>
#include <algorithm>
#include <string>
#include <limits>

class TemperatureListener : public rclcpp::Node
{
public:
    TemperatureListener()
    : Node("temperature_listener"),
      min_temp_(std::numeric_limits<double>::max()),
      max_temp_(std::numeric_limits<double>::lowest())
    {
        // Match publisher QoS exactly
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::TransientLocal);

        subscription_ = this->create_subscription<custom_temperature_msgs::msg::TemperatureReading>(
            "temperature_reading",
            qos_profile,
            std::bind(&TemperatureListener::callback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<custom_temperature_msgs::msg::TemperatureReading>::SharedPtr subscription_;
    std::deque<double> last_readings_;
    double min_temp_;
    double max_temp_;

    std::string unit_to_string(uint8_t unit_enum)
    {
        using custom_temperature_msgs::msg::TemperatureReading;
        switch (unit_enum)
        {
            case TemperatureReading::CELSIUS:    return "°C";
            case TemperatureReading::FAHRENHEIT: return "°F";
            default:                             return "unknown unit";
        }
    }

    void callback(const custom_temperature_msgs::msg::TemperatureReading::SharedPtr msg)
    {
        double temp = msg->temperature;
        std::string unit_str = unit_to_string(msg->unit);

        // Store last 10 readings
        last_readings_.push_back(temp);
        if (last_readings_.size() > 10) {
            last_readings_.pop_front();
        }

        // Update min/max over time
        min_temp_ = std::min(min_temp_, temp);
        max_temp_ = std::max(max_temp_, temp);

        // Calculate moving average
        double avg = 0.0;
        for (double t : last_readings_) avg += t;
        avg /= last_readings_.size();

        // Detect trend (last two readings)
        std::string trend = "stable";
        if (last_readings_.size() >= 2) {
            double diff = last_readings_.back() - *(last_readings_.rbegin() + 1);
            if (diff > 0.05) trend = "rising";
            else if (diff < -0.05) trend = "falling";
        }

        // Display readings
        RCLCPP_INFO(this->get_logger(),
            "Temp: %.2f%s | Avg(10): %.2f%s | Min: %.2f%s | Max: %.2f%s | Trend: %s",
            temp, unit_str.c_str(),
            avg, unit_str.c_str(),
            min_temp_, unit_str.c_str(),
            max_temp_, unit_str.c_str(),
            trend.c_str());

        // Warning system (normal range 35.5°C – 37.5°C)
        if (msg->unit == custom_temperature_msgs::msg::TemperatureReading::CELSIUS &&
            (temp < 35.5 || temp > 37.5))
        {
            RCLCPP_WARN(this->get_logger(),
                "WARNING: Temperature out of normal range! (%.2f%s)", temp, unit_str.c_str());
        }
        else if (msg->unit == custom_temperature_msgs::msg::TemperatureReading::FAHRENHEIT &&
                 (temp < 95.9 || temp > 99.5))
        {
            RCLCPP_WARN(this->get_logger(),
                "WARNING: Temperature out of normal range! (%.2f%s)", temp, unit_str.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperatureListener>());
    rclcpp::shutdown();
    return 0;
}
