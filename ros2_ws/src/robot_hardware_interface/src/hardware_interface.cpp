#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class MinimalSubscriber: public rclcpp::Node
{    
    public:
        MinimalSubscriber(): Node("hardware_interface")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Float64>(
                "wheel_l", 1, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
        }

    private:
        void topic_callback(const std_msgs::msg::Float64 msg)
        {
            RCLCPP_INFO(this->get_logger(), "Forwarding: %.2f", msg.data);   
        }

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
}