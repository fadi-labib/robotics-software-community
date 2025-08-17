#include "rclcpp/rclcpp.hpp"
#include "temperature_interfaces/msg/temperature.hpp"

#include <queue>
#include <cfloat>
#include <string>

class TemperatureSubscriberNode : public rclcpp::Node
{
public:
    TemperatureSubscriberNode() : Node("temperature_sub")
    {
        // Parameter declaration and default values
        this->declare_parameter("moving_average_period",10);
        this->declare_parameter("warning_max",50.0);
        this->declare_parameter("warning_min",10.0);

        // Assigning parameters to internal variables
        moving_average_period_ = this->get_parameter("moving_average_period").as_int();
        warning_max_ =this->get_parameter("warning_max").as_double();
        warning_min_ =this->get_parameter("warning_min").as_double();

        // Subscribe to temperature topic with sensor data QOS profile
        subscriber_ = this->create_subscription<temperature_interfaces::msg::Temperature>(
            "temperature",rclcpp::SensorDataQoS(),
            std::bind(&TemperatureSubscriberNode::temperature_processing, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Temperature subscriber has been started.");
    }

private:

    rclcpp::Subscription<temperature_interfaces::msg::Temperature>::SharedPtr subscriber_;
    // Parameters
    int moving_average_period_ ;
    double warning_max_;
    double warning_min_;
    
    // internal Variables

    // initial max temp = "-infinity"
    double max_temp_ = -DBL_MAX;;
    double sum_ = 0;
    double average_ = 0;
    std::queue<double> temp_celsius_values_;
    std::queue<double> last_averages_;

    // Threshold still needs tweaking to make the trend accurate
    double threshold_trend_ = 0.4;


    void temperature_processing(const temperature_interfaces::msg::Temperature::SharedPtr msg)
    {
        double temp_celsius = msg->value;
        
        // Convert to celsius if needed
        if (!msg->is_celsius)
        {
            temp_celsius = (temp_celsius - 32) * 5 / 9;
        }

        // Update max temperature
        if (temp_celsius > max_temp_)
        {
            max_temp_ = temp_celsius;
        }

        temp_celsius_values_.push(temp_celsius);

        // Calculate moving average
        sum_ += temp_celsius;
        double mv_average = 0;
        bool mv_average_ready = false;

        if (temp_celsius_values_.size() >= moving_average_period_)
        {   
      
            sum_ = sum_ - temp_celsius_values_.front();
            temp_celsius_values_.pop();
            
            mv_average = sum_ / moving_average_period_;
            mv_average_ready = true;

            // Add to queue to be used in Trend calculation
            last_averages_.push(mv_average);
        }

        // Calculate Temperature Trend from moving averages
        std::string temp_trend_str = "STABLE";

        if (last_averages_.size() == 2) 
        {
            double difference = last_averages_.back() - last_averages_.front();
            last_averages_.pop();

            if (difference > threshold_trend_)
            {
                temp_trend_str = "RISING";   
            }
            else if (difference < -threshold_trend_)
            {
                temp_trend_str = "FALLING";
            }
        }

        // Print Values
        if (!mv_average_ready)
        {
            RCLCPP_INFO(this->get_logger(), "\nTemperature received: %.2f°C\nNot enough data to calculate moving average.\nMax Temperature: %.2f°C\nWaiting for Moving average to calculate Temeprature trend.",temp_celsius,max_temp_);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "\nTemperature received: %.2f°C\nMoving average: %.2f°C\nMax Temperature: %.2f°C\nTemperature trend: %s.",temp_celsius,mv_average,max_temp_,temp_trend_str.c_str());
        }
        
        // Warning system
        if ((temp_celsius > warning_max_) || (temp_celsius < warning_min_) )
        {
            RCLCPP_WARN(this->get_logger(),"Temperature value outside normal range.");
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