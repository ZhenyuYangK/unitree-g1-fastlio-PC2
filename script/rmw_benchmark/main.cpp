#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

// No nodes, discovery, network traffic, or robot API calls.
int main()
{
  livox_ros_driver2::msg::CustomMsg message;
  message.point_num = 20000;
  message.points.resize(message.point_num);
  message.points.front().x = 1.25f;
  message.points.back().offset_time = 99999999;
  rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> codec;
  rclcpp::SerializedMessage bytes;
  std::vector<double> times;
  for (int i = 0; i < 12; ++i) {
    const auto start = std::chrono::steady_clock::now();
    codec.serialize_message(&message, &bytes);
    const double elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    if (i >= 2) {
      times.push_back(elapsed);
    }
  }
  livox_ros_driver2::msg::CustomMsg copy;
  codec.deserialize_message(&bytes, &copy);
  if (copy.points.size() != message.points.size() || copy.points.front().x != 1.25f ||
    copy.points.back().offset_time != 99999999)
  {
    std::cerr << "Serialization roundtrip failed\n";
    return 1;
  }
  std::sort(times.begin(), times.end());
  std::cout << "points=20000 bytes=" << bytes.size()
            << " median_ms=" << times[times.size() / 2]
            << " min_ms=" << times.front() << " max_ms=" << times.back()
            << " roundtrip=PASS\n";
}
