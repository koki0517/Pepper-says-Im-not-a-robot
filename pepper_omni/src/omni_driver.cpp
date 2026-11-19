// cmd_vel (geometry_msgs/Twist) -> 3-wheel omni base driven by Dynamixel MX-106
// (Protocol 1.0, wheel mode).
//
// Pepper base frame: X+ forward, Y+ left, Z+ up. Wheel mount angles are
// measured from X+ toward Y+ (counter-clockwise seen from above).
//
// Scale chain:
//   cmd_vel [m/s, rad/s]
//     -> wheel surface speed [m/s]   v_i = -sin(a_i) vx + cos(a_i) vy + L wz
//     -> wheel angular speed [rad/s] w_i = v_i / r
//     -> rpm                         w_i * 60 / (2 pi)
//     -> Moving Speed register       |rpm| / 0.114  (0..1023, bit10 = CW)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
// MX-106 control table, Protocol 1.0
constexpr uint16_t ADDR_TORQUE_ENABLE = 24;
constexpr uint16_t ADDR_MOVING_SPEED = 32;
constexpr uint16_t LEN_MOVING_SPEED = 2;
constexpr double RPM_PER_UNIT = 0.114;   // Moving Speed unit in wheel mode
constexpr int SPEED_VALUE_MAX = 1023;    // magnitude bits
constexpr int SPEED_DIR_CW_BIT = 1024;   // bit10: 0 = CCW, 1 = CW
constexpr double PROTOCOL_VERSION = 1.0;
}  // namespace

class OmniDriver : public rclcpp::Node
{
public:
  OmniDriver()
  : Node("omni_driver")
  {
    port_name_ = declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 1000000);

    auto ids = declare_parameter<std::vector<int64_t>>("wheel_ids", {3, 1, 2});
    auto angles = declare_parameter<std::vector<double>>(
      "wheel_angles_deg", {-60.0, 180.0, 60.0});
    // +1: motor CCW (seen from the horn) moves the wheel contact point in the
    //     CCW tangent direction of the base. -1: opposite.
    auto dirs = declare_parameter<std::vector<double>>(
      "motor_directions", {-1.0, -1.0, -1.0});

    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.05);   // m
    base_radius_ = declare_parameter<double>("base_radius", 0.12);     // m, center to wheel
    max_speed_value_ = declare_parameter<int>("max_speed_value", 400); // register units
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);      // s
    const double rate = declare_parameter<double>("control_rate", 50.0);  // Hz

    if (ids.size() != angles.size() || ids.size() != dirs.size()) {
      throw std::runtime_error("wheel_ids, wheel_angles_deg, motor_directions must have same size");
    }
    max_speed_value_ = std::clamp(max_speed_value_, 0, SPEED_VALUE_MAX);
    for (size_t i = 0; i < ids.size(); ++i) {
      wheels_.push_back({static_cast<uint8_t>(ids[i]), angles[i] * M_PI / 180.0, dirs[i]});
    }

    openPort();
    for (const auto & w : wheels_) {
      writeTorque(w.id, true);
    }
    writeSpeeds(std::vector<int>(wheels_.size(), 0));

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_ = *msg;
        last_cmd_time_ = now();
      });
    last_cmd_time_ = now();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&OmniDriver::onTimer, this));

    RCLCPP_INFO(
      get_logger(), "omni_driver ready on %s @ %d bps (max_speed_value=%d, %.1f rpm)",
      port_name_.c_str(), baud_rate_, max_speed_value_, max_speed_value_ * RPM_PER_UNIT);
  }

  ~OmniDriver() override
  {
    // Stop wheels and release torque on exit.
    writeSpeeds(std::vector<int>(wheels_.size(), 0));
    for (const auto & w : wheels_) {
      writeTorque(w.id, false);
    }
    port_->closePort();
  }

private:
  struct Wheel
  {
    uint8_t id;
    double angle;      // rad
    double direction;  // +1 / -1
  };

  void openPort()
  {
    port_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!port_->openPort()) {
      throw std::runtime_error("failed to open " + port_name_);
    }
    if (!port_->setBaudRate(baud_rate_)) {
      throw std::runtime_error("failed to set baud rate");
    }
    sync_write_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_, packet_, ADDR_MOVING_SPEED, LEN_MOVING_SPEED);
  }

  void writeTorque(uint8_t id, bool enable)
  {
    uint8_t err = 0;
    int res = packet_->write1ByteTxRx(port_, id, ADDR_TORQUE_ENABLE, enable ? 1 : 0, &err);
    if (res != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "id %d torque: %s", id, packet_->getTxRxResult(res));
    } else if (err != 0) {
      RCLCPP_ERROR(get_logger(), "id %d torque: %s", id, packet_->getRxPacketError(err));
    }
  }

  // Signed speed in register units (-1023..1023) -> Protocol 1.0 wheel mode value.
  static uint16_t encodeSpeed(int signed_value)
  {
    if (signed_value >= 0) {
      return static_cast<uint16_t>(signed_value);  // CCW
    }
    return static_cast<uint16_t>(-signed_value | SPEED_DIR_CW_BIT);  // CW
  }

  void writeSpeeds(const std::vector<int> & values)
  {
    sync_write_->clearParam();
    for (size_t i = 0; i < wheels_.size(); ++i) {
      const uint16_t v = encodeSpeed(values[i]);
      uint8_t data[2] = {DXL_LOBYTE(v), DXL_HIBYTE(v)};
      sync_write_->addParam(wheels_[i].id, data);
    }
    int res = sync_write_->txPacket();
    if (res != COMM_SUCCESS) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "sync write: %s", packet_->getTxRxResult(res));
    }
  }

  void onTimer()
  {
    geometry_msgs::msg::Twist cmd;
    if ((now() - last_cmd_time_).seconds() <= cmd_timeout_) {
      cmd = cmd_;
    }

    // cmd_vel -> signed register value per wheel (float)
    std::vector<double> raw(wheels_.size());
    double peak = 0.0;
    for (size_t i = 0; i < wheels_.size(); ++i) {
      const auto & w = wheels_[i];
      const double v_surface = -std::sin(w.angle) * cmd.linear.x +
        std::cos(w.angle) * cmd.linear.y + base_radius_ * cmd.angular.z;  // m/s
      const double omega = v_surface / wheel_radius_;                      // rad/s
      const double rpm = omega * 60.0 / (2.0 * M_PI);
      raw[i] = w.direction * rpm / RPM_PER_UNIT;
      peak = std::max(peak, std::abs(raw[i]));
    }

    // If any wheel exceeds the limit, scale all wheels down together so the
    // direction of motion is preserved.
    const double scale = (peak > max_speed_value_) ? max_speed_value_ / peak : 1.0;

    std::vector<int> values(wheels_.size());
    for (size_t i = 0; i < wheels_.size(); ++i) {
      values[i] = static_cast<int>(std::lround(raw[i] * scale));
    }
    writeSpeeds(values);
  }

  std::string port_name_;
  int baud_rate_;
  double wheel_radius_, base_radius_, cmd_timeout_;
  int max_speed_value_;
  std::vector<Wheel> wheels_;

  dynamixel::PortHandler * port_ = nullptr;
  dynamixel::PacketHandler * packet_ = nullptr;
  std::unique_ptr<dynamixel::GroupSyncWrite> sync_write_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OmniDriver>();
  rclcpp::spin(node);
  node.reset();  // run destructor (stop + torque off) before shutdown
  rclcpp::shutdown();
  return 0;
}
