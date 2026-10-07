#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

using std::placeholders::_1;

class LidarRawToScanNode : public rclcpp::Node
{
public:
  LidarRawToScanNode()
  : Node("lidar_raw_to_scan")
  {
    input_topic_0_ = declare_parameter<std::string>("input_topic_0", "/rover_lidar_raw_0");
    input_topic_1_ = declare_parameter<std::string>("input_topic_1", "/rover_lidar_raw_1");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    frame_id_ = declare_parameter<std::string>("frame_id", "laser_frame");
    scan_points_ = declare_parameter<int>("scan_points", 360);
    chunk_points_ = declare_parameter<int>("chunk_points", 180);
    range_min_ = declare_parameter<double>("range_min", 0.12);
    range_max_ = declare_parameter<double>("range_max", 8.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.125);
    stamp_offset_ = declare_parameter<double>("stamp_offset", -0.125);
    reverse_ = declare_parameter<bool>("reverse", true);

    auto raw_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, scan_qos);
    raw_sub_0_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      input_topic_0_, raw_qos, std::bind(&LidarRawToScanNode::raw0_callback, this, _1));
    raw_sub_1_ = create_subscription<std_msgs::msg::UInt16MultiArray>(
      input_topic_1_, raw_qos, std::bind(&LidarRawToScanNode::raw1_callback, this, _1));
  }

private:
  void raw0_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    chunk_0_ = make_chunk(msg);
    have_chunk_0_ = true;
    publish_if_ready();
  }

  void raw1_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    chunk_1_ = make_chunk(msg);
    have_chunk_1_ = true;
    publish_if_ready();
  }

  std::vector<uint16_t> make_chunk(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) const
  {
    std::vector<uint16_t> chunk(chunk_points_, 0);
    const auto copy_size = std::min<std::size_t>(msg->data.size(), chunk.size());
    std::copy_n(msg->data.begin(), copy_size, chunk.begin());
    return chunk;
  }

  void publish_if_ready()
  {
    if (!have_chunk_0_ || !have_chunk_1_ || scan_points_ <= 0) {
      return;
    }

    std::vector<uint16_t> raw;
    raw.reserve(static_cast<std::size_t>(scan_points_));
    raw.insert(raw.end(), chunk_0_.begin(), chunk_0_.end());
    raw.insert(raw.end(), chunk_1_.begin(), chunk_1_.end());
    raw.resize(static_cast<std::size_t>(scan_points_), 0);

    have_chunk_0_ = false;
    have_chunk_1_ = false;

    if (reverse_) {
      std::reverse(raw.begin(), raw.end());
    }

    constexpr double pi = 3.14159265358979323846;
    const float angle_increment =
      static_cast<float>((2.0 * pi) / static_cast<double>(scan_points_));

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now() + rclcpp::Duration::from_seconds(stamp_offset_);
    scan.header.frame_id = frame_id_;
    scan.angle_min = 0.0F;
    scan.angle_max = static_cast<float>(scan_points_ - 1) * angle_increment;
    scan.angle_increment = angle_increment;
    scan.time_increment = scan_points_ > 0 ?
      static_cast<float>(scan_time_ / static_cast<double>(scan_points_)) : 0.0F;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges.reserve(raw.size());

    for (const uint16_t distance_mm : raw) {
      const float distance_m = static_cast<float>(distance_mm) / 1000.0F;
      if (distance_m >= scan.range_min && distance_m <= scan.range_max) {
        scan.ranges.push_back(distance_m);
      } else {
        scan.ranges.push_back(std::numeric_limits<float>::infinity());
      }
    }

    scan_pub_->publish(scan);
    if (!published_first_scan_) {
      RCLCPP_INFO(
        get_logger(),
        "Publishing %zu-point LaserScan on %s with frame_id %s",
        scan.ranges.size(),
        output_topic_.c_str(),
        frame_id_.c_str());
      published_first_scan_ = true;
    }
  }

  std::string input_topic_0_;
  std::string input_topic_1_;
  std::string output_topic_;
  std::string frame_id_;
  int scan_points_;
  int chunk_points_;
  double range_min_;
  double range_max_;
  double scan_time_;
  double stamp_offset_;
  bool reverse_;

  bool have_chunk_0_ = false;
  bool have_chunk_1_ = false;
  bool published_first_scan_ = false;
  std::vector<uint16_t> chunk_0_;
  std::vector<uint16_t> chunk_1_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr raw_sub_0_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr raw_sub_1_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarRawToScanNode>());
  rclcpp::shutdown();
  return 0;
}
