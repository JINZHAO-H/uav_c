#include <array>
#include <chrono>
#include <cmath>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{

rclcpp::QoS px4_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .transient_local();
}

std::array<std::array<double, 3>, 3> quaternion_to_matrix(
  double qx, double qy, double qz, double qw)
{
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (norm <= 1e-9) {
    return {{
      {{1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}},
      {{0.0, 0.0, 1.0}},
    }};
  }

  const double x = qx / norm;
  const double y = qy / norm;
  const double z = qz / norm;
  const double w = qw / norm;

  return {{
    {{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)}},
    {{2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)}},
    {{2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}},
  }};
}

std::array<std::array<double, 3>, 3> matrix_multiply(
  const std::array<std::array<double, 3>, 3> & a,
  const std::array<std::array<double, 3>, 3> & b)
{
  std::array<std::array<double, 3>, 3> out{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      out[row][col] =
        a[row][0] * b[0][col] +
        a[row][1] * b[1][col] +
        a[row][2] * b[2][col];
    }
  }
  return out;
}

std::array<float, 4> matrix_to_quaternion_wxyz(
  const std::array<std::array<double, 3>, 3> & m)
{
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  const double trace = m[0][0] + m[1][1] + m[2][2];

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m[2][1] - m[1][2]) / s;
    y = (m[0][2] - m[2][0]) / s;
    z = (m[1][0] - m[0][1]) / s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    w = (m[2][1] - m[1][2]) / s;
    x = 0.25 * s;
    y = (m[0][1] + m[1][0]) / s;
    z = (m[0][2] + m[2][0]) / s;
  } else if (m[1][1] > m[2][2]) {
    const double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    w = (m[0][2] - m[2][0]) / s;
    x = (m[0][1] + m[1][0]) / s;
    y = 0.25 * s;
    z = (m[1][2] + m[2][1]) / s;
  } else {
    const double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    w = (m[1][0] - m[0][1]) / s;
    x = (m[0][2] + m[2][0]) / s;
    y = (m[1][2] + m[2][1]) / s;
    z = 0.25 * s;
  }

  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (norm <= 1e-9) {
    return {{1.0F, 0.0F, 0.0F, 0.0F}};
  }

  return {{
    static_cast<float>(w / norm),
    static_cast<float>(x / norm),
    static_cast<float>(y / norm),
    static_cast<float>(z / norm),
  }};
}

std::array<float, 4> orientation_enu_flu_to_ned_frd(
  double qx, double qy, double qz, double qw)
{
  const std::array<std::array<double, 3>, 3> enu_to_ned{{
    {{0.0, 1.0, 0.0}},
    {{1.0, 0.0, 0.0}},
    {{0.0, 0.0, -1.0}},
  }};

  const std::array<std::array<double, 3>, 3> frd_to_flu{{
    {{1.0, 0.0, 0.0}},
    {{0.0, -1.0, 0.0}},
    {{0.0, 0.0, -1.0}},
  }};

  const auto r_enu_flu = quaternion_to_matrix(qx, qy, qz, qw);
  const auto r_ned_flu = matrix_multiply(enu_to_ned, r_enu_flu);
  const auto r_ned_frd = matrix_multiply(r_ned_flu, frd_to_flu);

  return matrix_to_quaternion_wxyz(r_ned_frd);
}

bool finite3(const std::array<float, 3> & values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

}  // namespace

class LocationBridge final : public rclcpp::Node
{
public:
  LocationBridge()
  : Node("location_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/aft_mapped_to_init");
    output_topic_ = declare_parameter<std::string>(
      "visual_odometry_topic", "/fmu/in/vehicle_visual_odometry");

    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    origin_z_ = declare_parameter<double>("origin_z", 0.0);

    zero_initial_xy_ = declare_parameter<bool>("zero_initial_xy", true);
    zero_initial_z_ = declare_parameter<bool>("zero_initial_z", true);

    const double publish_rate = declare_parameter<double>("publish_rate", 30.0);
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.5);

    position_variance_ = declare_parameter<double>("position_variance", 0.03);
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.05);
    velocity_variance_ = declare_parameter<double>("velocity_variance", 0.05);

    odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(output_topic_, px4_qos());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LocationBridge::odometry_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate, 1.0));
    republish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocationBridge::republish_latest, this));

    RCLCPP_INFO(
      get_logger(),
      "LocationBridge started: %s -> %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  uint64_t now_us()
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;
    const double raw_z = msg->pose.pose.position.z;

    if (!xy_offset_ready_) {
      offset_x_ = origin_x_;
      offset_y_ = origin_y_;

      if (zero_initial_xy_) {
        offset_x_ -= raw_x;
        offset_y_ -= raw_y;
      }

      xy_offset_ready_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Initial x/y offset set: raw=(%.3f, %.3f), offset=(%.3f, %.3f)",
        raw_x,
        raw_y,
        offset_x_,
        offset_y_);
    }

    if (!z_offset_ready_) {
      offset_z_ = origin_z_;

      if (zero_initial_z_) {
        offset_z_ -= raw_z;
      }

      z_offset_ready_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Initial z offset set: raw=%.3f, offset=%.3f",
        raw_z,
        offset_z_);
    }

    const double x_enu = raw_x + offset_x_;
    const double y_enu = raw_y + offset_y_;
    const double z_enu = raw_z + offset_z_;

    const auto & q = msg->pose.pose.orientation;
    const auto & twist = msg->twist.twist;

    px4_msgs::msg::VehicleOdometry odom{};
    odom.timestamp = now_us();
    odom.timestamp_sample = odom.timestamp;

    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odom.position = {{
      static_cast<float>(y_enu),
      static_cast<float>(x_enu),
      static_cast<float>(-z_enu),
    }};

    odom.q = orientation_enu_flu_to_ned_frd(q.x, q.y, q.z, q.w);

    odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    odom.velocity = {{
      static_cast<float>(twist.linear.y),
      static_cast<float>(twist.linear.x),
      static_cast<float>(-twist.linear.z),
    }};

    odom.angular_velocity = {{
      static_cast<float>(twist.angular.x),
      static_cast<float>(-twist.angular.y),
      static_cast<float>(-twist.angular.z),
    }};

    odom.position_variance = {{
      static_cast<float>(position_variance_),
      static_cast<float>(position_variance_),
      static_cast<float>(position_variance_),
    }};

    odom.orientation_variance = {{
      static_cast<float>(orientation_variance_),
      static_cast<float>(orientation_variance_),
      static_cast<float>(orientation_variance_),
    }};

    odom.velocity_variance = {{
      static_cast<float>(velocity_variance_),
      static_cast<float>(velocity_variance_),
      static_cast<float>(velocity_variance_),
    }};

    odom.reset_counter = 0;
    odom.quality = 100;

    if (!finite3(odom.position) || !finite3(odom.velocity)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring non-finite odometry sample from %s",
        input_topic_.c_str());
      return;
    }

    latest_odom_ = odom;
    have_latest_odom_ = true;
    last_odom_time_ = now();

    odom_pub_->publish(odom);
  }

  void republish_latest()
  {
    if (!have_latest_odom_) {
      return;
    }

    const double age = (now() - last_odom_time_).seconds();

    if (odometry_timeout_ > 0.0 && age > odometry_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "No fresh %s for %.2fs; stop republishing PX4 visual odometry",
        input_topic_.c_str(),
        age);
      return;
    }

    latest_odom_.timestamp = now_us();
    odom_pub_->publish(latest_odom_);
  }

  std::string input_topic_;
  std::string output_topic_;

  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};

  bool zero_initial_xy_{true};
  bool zero_initial_z_{true};

  bool xy_offset_ready_{false};
  bool z_offset_ready_{false};

  double offset_x_{0.0};
  double offset_y_{0.0};
  double offset_z_{0.0};

  double position_variance_{0.03};
  double orientation_variance_{0.05};
  double velocity_variance_{0.05};
  double odometry_timeout_{0.5};

  bool have_latest_odom_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  px4_msgs::msg::VehicleOdometry latest_odom_{};

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocationBridge>());
  rclcpp::shutdown();
  return 0;
}
