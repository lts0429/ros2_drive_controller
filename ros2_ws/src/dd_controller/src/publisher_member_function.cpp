#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dd_controller/differential_kinematics.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(): Node("twist_subscriber")
    {
      wheel_base_ = this->declare_parameter<double>("wheel_base", 0.5);
      wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.1);
      left_wheels_ = this->declare_parameter<std::vector<std::string>>("left_wheels", std::vector<std::string>{});
      right_wheels_ = this->declare_parameter<std::vector<std::string>>("right_wheels", std::vector<std::string>{});

      RCLCPP_INFO(this->get_logger(), "Wheelbase set to %.2f meters", wheel_base_);

      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      
      wheels_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("wheel_velocities", 1);

    }

  private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
      // double left_speed = 0.0, right_speed = 0.0;

      auto left_speed = std_msgs::msg::Float64();
      auto right_speed = std_msgs::msg::Float64();

      // Call the function to calculate wheel speeds based on the Twist message
      differential_kinematics::calculate_wheel_speeds(msg, wheel_radius_, wheel_base_, left_speed, right_speed);

      std_msgs::msg::Float64MultiArray velocities_msg;

      // Clear existing array
      velocities_msg.data.clear();
    
      // Add left wheels' speeds
      for (size_t i = 0; i < left_wheels_.size(); ++i) {
        velocities_msg.data.push_back(left_speed.data);
      }
    
      // Add right wheels' speeds
      for (size_t i = 0; i < right_wheels_.size(); ++i) {
        velocities_msg.data.push_back(right_speed.data);
      }

      // Publish
      wheels_publisher_->publish(velocities_msg);
      
      // Print the left and right speeds
      RCLCPP_INFO(this->get_logger(), "Left speed: %.2f, Right speed: %.2f", left_speed.data, right_speed.data);
    }
  
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::vector<std::string> left_wheels_;
    std::vector<std::string> right_wheels_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheels_publisher_;
    double wheel_base_;
    double wheel_radius_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}