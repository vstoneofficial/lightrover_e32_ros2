#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class DroverGamepad : public rclcpp::Node
{
public:
  DroverGamepad()
  : Node("lightrover_e32_gamepad")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    output_topic_ = declare_parameter<std::string>("output_topic", "/lightrover_e32_teleop_raw");
    linear_axis_ = declare_parameter<int>("linear_axis", 1);
    angular_axis_ = declare_parameter<int>("angular_axis", 3);
    dpad_linear_axis_ = declare_parameter<int>("dpad_linear_axis", 7);
    dpad_angular_axis_ = declare_parameter<int>("dpad_angular_axis", 6);
    enable_button_ = declare_parameter<int>("enable_button", 4);
    turbo_button_ = declare_parameter<int>("turbo_button", 5);
    slow_button_ = declare_parameter<int>("slow_button", 6);
    invert_linear_ = declare_parameter<bool>("invert_linear", false);
    invert_angular_ = declare_parameter<bool>("invert_angular", false);
    deadzone_ = declare_parameter<double>("deadzone", 0.08);
    normal_scale_ = declare_parameter<double>("normal_scale", 1.0);
    turbo_scale_ = declare_parameter<double>("turbo_scale", 1.0);
    slow_scale_ = declare_parameter<double>("slow_scale", 0.35);
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);

    if (publish_rate_ <= 0.0) {
      publish_rate_ = 20.0;
    }
    if (command_timeout_ <= 0.0) {
      command_timeout_ = 0.5;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&DroverGamepad::joy_callback, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DroverGamepad::timer_callback, this));

    last_joy_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Drover gamepad: %s -> %s, linear_axis=%d, angular_axis=%d, enable_button=%d",
      joy_topic_.c_str(),
      output_topic_.c_str(),
      linear_axis_,
      angular_axis_,
      enable_button_);
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = now();
    last_command_ = geometry_msgs::msg::Twist();

    if (!button_pressed(*msg, enable_button_)) {
      return;
    }

    double scale = normal_scale_;
    if (button_pressed(*msg, turbo_button_)) {
      scale = turbo_scale_;
    } else if (button_pressed(*msg, slow_button_)) {
      scale = slow_scale_;
    }

    const double linear = axis_value(*msg, linear_axis_);
    const double angular = axis_value(*msg, angular_axis_);
    const double dpad_linear = axis_value(*msg, dpad_linear_axis_);
    const double dpad_angular = axis_value(*msg, dpad_angular_axis_);

    const double linear_direction = invert_linear_ ? -1.0 : 1.0;
    const double angular_direction = invert_angular_ ? -1.0 : 1.0;

    last_command_.linear.x = select_axis(linear, dpad_linear) * scale * linear_direction;
    last_command_.angular.z = select_axis(angular, dpad_angular) * scale * angular_direction;
  }

  void timer_callback()
  {
    auto command = last_command_;
    if ((now() - last_joy_time_).seconds() > command_timeout_) {
      command = geometry_msgs::msg::Twist();
    }
    twist_pub_->publish(command);
  }

  double axis_value(const sensor_msgs::msg::Joy & msg, int index) const
  {
    if (index < 0 || static_cast<std::size_t>(index) >= msg.axes.size()) {
      return 0.0;
    }

    const double value = msg.axes[static_cast<std::size_t>(index)];
    if (std::abs(value) < deadzone_) {
      return 0.0;
    }
    return std::clamp(value, -1.0, 1.0);
  }

  static bool button_pressed(const sensor_msgs::msg::Joy & msg, int index)
  {
    if (index < 0) {
      return true;
    }
    if (static_cast<std::size_t>(index) >= msg.buttons.size()) {
      return false;
    }
    return msg.buttons[static_cast<std::size_t>(index)] != 0;
  }

  static double select_axis(double stick_value, double dpad_value)
  {
    return std::abs(dpad_value) > 0.5 ? dpad_value : stick_value;
  }

  std::string joy_topic_;
  std::string output_topic_;
  int linear_axis_;
  int angular_axis_;
  int dpad_linear_axis_;
  int dpad_angular_axis_;
  int enable_button_;
  int turbo_button_;
  int slow_button_;
  bool invert_linear_;
  bool invert_angular_;
  double deadzone_;
  double normal_scale_;
  double turbo_scale_;
  double slow_scale_;
  double publish_rate_;
  double command_timeout_;

  geometry_msgs::msg::Twist last_command_;
  rclcpp::Time last_joy_time_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroverGamepad>());
  rclcpp::shutdown();
  return 0;
}
