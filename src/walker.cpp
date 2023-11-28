#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <chrono>

using namespace std::chrono_literals;

class WalkerAlgo : public rclcpp::Node {
 public:
  WalkerAlgo() : Node("walker") {
    laser_data_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&WalkerAlgo::laser_scan_cb, this, std::placeholders::_1));
    velcoity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

 private:
  void laser_scan_cb(const sensor_msgs::msg::LaserScan& laser_data) {
    if (laser_data.header.stamp.sec == 0) {
      return;
    }

    auto laser_scan = laser_data.ranges;
    for (unsigned int scan_angle = 330; scan_angle < 330 + 60; scan_angle++) {
      if (laser_scan[scan_angle % 360] < 0.8) {
        move_robot(0.0, 0.1);
      } else {
        move_robot(0.1, 0.0);
      }
    }
  }

  void move_robot(float x, float z) {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = x;
    twist.angular.z = -z;
    velcoity_publisher->publish(twist);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_data_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velcoity_publisher;
  rclcpp::TimerBase::SharedPtr timer;
  sensor_msgs::msg::LaserScan laser_scan;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkerAlgo>());
  rclcpp::shutdown();
  return 0;
}