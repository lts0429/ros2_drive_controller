#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "diff_drive_controller/differential_kinematics.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(): Node("twist_subscriber")
    {
      this->declare_parameter<double>("wheelbase", 0.5);
      this->get_parameter("wheelbase", wheelbase_);

      RCLCPP_INFO(this->get_logger(), "Wheelbase set to %.2f meters", wheelbase_);

      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      
      publisherLeft_ = this->create_publisher<std_msgs::msg::Float64>("wheel_l", 1);
      publisherRight_ = this->create_publisher<std_msgs::msg::Float64>("wheel_r", 1);
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      // double left_speed = 0.0, right_speed = 0.0;

      auto left_speed = std_msgs::msg::Float64();
      auto right_speed = std_msgs::msg::Float64();

      // Call the function to calculate wheel speeds based on the Twist message
      differential_kinematics::calculate_wheel_speeds(msg, wheelbase_, left_speed, right_speed);

      // Publish the individual speed
      publisherLeft_->publish(left_speed);
      publisherRight_->publish(right_speed);

      // Print the left and right speeds
      RCLCPP_INFO(this->get_logger(), "Left speed: %.2f, Right speed: %.2f", left_speed.data, right_speed.data);
    }
  
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisherLeft_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisherRight_;
    double wheelbase_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}