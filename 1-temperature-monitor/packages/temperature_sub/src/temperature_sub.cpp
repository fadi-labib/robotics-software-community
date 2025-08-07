#include "rclcpp/rclcpp.hpp"
#include "temperature_interfaces/msg/temperature.hpp"

#include <vector>
#include <cfloat>

class TemperatureSubscriberNode : public rclcpp::Node
{
public:
    TemperatureSubscriberNode() : Node("temperature_sub")
    {
        // Parameter declaration and default values
        this->declare_parameter("moving_average_period",10);

        // Assigning parameters to internal variables
        moving_average_period_ = this->get_parameter("moving_average_period").as_int();

        subscriber_ = this->create_subscription<temperature_interfaces::msg::Temperature>(
            "temperature", 10,
            std::bind(&TemperatureSubscriberNode::temperature_processing, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Temperature subscriber has been started.");
    }

private:

    rclcpp::Subscription<temperature_interfaces::msg::Temperature>::SharedPtr subscriber_;

    int moving_average_period_ ;

    void temperature_processing(const temperature_interfaces::msg::Temperature::SharedPtr msg)
    {
        double temp_celsius = msg->value;

        if (!msg->is_celsius)
        {
            // Convert to celsius
            temp_celsius = (temp_celsius - 32) * 5 / 9;
        }

        // initial max temp = "-infinity"
        static double s_max_temp = -DBL_MAX;

        if (temp_celsius > s_max_temp)
        {
            s_max_temp = temp_celsius;
        }

        static std::vector<double> s_temp_celsius_values;
        s_temp_celsius_values.push_back(temp_celsius);


        // Calculate moving average
        static int s_iterator = 0;
        double sum = 0;
        double average =0;

        if (s_temp_celsius_values.size() >= moving_average_period_)
        {
            for (s_iterator; s_iterator < s_temp_celsius_values.size(); s_iterator++)
            {
                sum += s_temp_celsius_values[s_iterator];
            }
            s_iterator = s_iterator - (moving_average_period_ - 1);

            average = sum / moving_average_period_;
        }

        if (s_temp_celsius_values.size() < moving_average_period_)
        {
            RCLCPP_INFO(this->get_logger(), "\nTemperature received: %.2f°C\nNot enough data to calculate moving average.\nMax Temperature: %.2f°C\n",temp_celsius,s_max_temp);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "\nTemperature received: %.2f°C\nMoving average: %.2f°C\nMax Temperature: %.2f°C\n",temp_celsius,average,s_max_temp);
        }
        
       

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TemperatureSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}