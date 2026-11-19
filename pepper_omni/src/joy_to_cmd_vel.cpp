// /spacenav/joy (sensor_msgs/Joy) -> cmd_vel (geometry_msgs/Twist)
//
// Pepper base frame: X+ forward, Y+ left, Z+ up (right-handed).
// angular.z is positive counter-clockwise when viewed from above.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

class JoyToCmdVel : public rclcpp::Node
{
public:
  JoyToCmdVel()
  : Node("joy_to_cmd_vel")
  {
    // Which joy axis drives each velocity component, and its sign.
    axis_x_ = declare_parameter<int>("axis_linear_x", 2);
    axis_y_ = declare_parameter<int>("axis_linear_y", 1);
    axis_yaw_ = declare_parameter<int>("axis_angular_z", 3);
    sign_x_ = declare_parameter<double>("sign_linear_x", 1.0);
    sign_y_ = declare_parameter<double>("sign_linear_y", 1.0);
    sign_yaw_ = declare_parameter<double>("sign_angular_z", 1.0);

    // Velocity at full deflection.
    max_x_ = declare_parameter<double>("max_linear_x", 0.3);      // m/s
    max_y_ = declare_parameter<double>("max_linear_y", 0.3);      // m/s
    max_yaw_ = declare_parameter<double>("max_angular_z", 1.0);   // rad/s

    deadzone_ = declare_parameter<double>("deadzone", 0.1);
    timeout_ = declare_parameter<double>("timeout", 0.5);         // s

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoyToCmdVel::onJoy, this, std::placeholders::_1));
    watchdog_ = create_wall_timer(100ms, std::bind(&JoyToCmdVel::onWatchdog, this));

    last_joy_time_ = now();
  }

private:
  // Apply deadzone and rescale so output starts from 0 at the deadzone edge.
  double shape(const sensor_msgs::msg::Joy & joy, int axis) const
  {
    if (axis < 0 || axis >= static_cast<int>(joy.axes.size())) {
      return 0.0;
    }
    const double v = std::clamp(static_cast<double>(joy.axes[axis]), -1.0, 1.0);
    if (std::abs(v) <= deadzone_) {
      return 0.0;
    }
    return std::copysign((std::abs(v) - deadzone_) / (1.0 - deadzone_), v);
  }

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    last_joy_time_ = now();
    stopped_ = false;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = sign_x_ * max_x_ * shape(*joy, axis_x_);
    twist.linear.y = sign_y_ * max_y_ * shape(*joy, axis_y_);
    twist.angular.z = sign_yaw_ * max_yaw_ * shape(*joy, axis_yaw_);
    pub_->publish(twist);
  }

  // Publish zero once if joy input stops arriving.
  void onWatchdog()
  {
    if (stopped_) {
      return;
    }
    if ((now() - last_joy_time_).seconds() > timeout_) {
      RCLCPP_WARN(get_logger(), "joy timeout: publishing zero velocity");
      pub_->publish(geometry_msgs::msg::Twist());
      stopped_ = true;
    }
  }

  int axis_x_, axis_y_, axis_yaw_;
  double sign_x_, sign_y_, sign_yaw_;
  double max_x_, max_y_, max_yaw_;
  double deadzone_, timeout_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::Time last_joy_time_;
  bool stopped_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToCmdVel>());
  rclcpp::shutdown();
  return 0;
}
