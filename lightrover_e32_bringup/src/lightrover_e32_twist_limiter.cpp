#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DroverTwistLimiter : public rclcpp::Node
{
public:
  DroverTwistLimiter()
  : Node("lightrover_e32_twist_limiter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lightrover_e32_teleop_raw");
    output_topic_ = declare_parameter<std::string>("output_topic", "/rover_twist");
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.10);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 1.0);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);

    if (publish_rate_ <= 0.0) {
      publish_rate_ = 20.0;
    }
    if (command_timeout_ <= 0.0) {
      command_timeout_ = 0.5;
    }

    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic_,
      10,
      std::bind(&DroverTwistLimiter::twist_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DroverTwistLimiter::timer_callback, this));

    last_command_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Limiting %s -> %s: linear.x <= %.3f m/s, angular.z <= %.3f rad/s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      max_linear_x_,
      max_angular_z_);
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_command_.linear.x = clamp(msg->linear.x, max_linear_x_);
    last_command_.linear.y = 0.0;
    last_command_.linear.z = 0.0;
    last_command_.angular.x = 0.0;
    last_command_.angular.y = 0.0;
    last_command_.angular.z = clamp(msg->angular.z, max_angular_z_);
    last_command_time_ = now();
  }

  void timer_callback()
  {
    auto command = last_command_;
    if ((now() - last_command_time_).seconds() > command_timeout_) {
      command = geometry_msgs::msg::Twist();
    }
    pub_->publish(command);
  }

  static double clamp(double value, double limit)
  {
    const double abs_limit = std::abs(limit);
    return std::clamp(value, -abs_limit, abs_limit);
  }

  std::string input_topic_;
  std::string output_topic_;
  double max_linear_x_;
  double max_angular_z_;
  double publish_rate_;
  double command_timeout_;

  geometry_msgs::msg::Twist last_command_;
  rclcpp::Time last_command_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroverTwistLimiter>());
  rclcpp::shutdown();
  return 0;
}
