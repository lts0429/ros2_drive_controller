#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "diff_drive_controller/differential_kinematics.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(): Node("twist_subscriber")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

      // Wheelbase of the robot (in meters)
      wheelbase_ = 0.5;  // Example: 50 cm between the wheels
    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      double left_speed = 0.0, right_speed = 0.0;

      // Call the function to calculate wheel speeds based on the Twist message
      differential_kinematics::calculate_wheel_speeds(msg, wheelbase_, left_speed, right_speed);

      // Print the left and right speeds
      RCLCPP_INFO(this->get_logger(), "Left speed: %.2f, Right speed: %.2f", left_speed, right_speed);
    }
  
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    double wheelbase_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}