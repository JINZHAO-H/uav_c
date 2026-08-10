#include <array>  //用固定长度数组存 3x3 矩阵、四元数、位置等
#include <chrono>  //用于定时器周期
#include <cmath>  //用于 sqrt、isfinite 等数学函数
#include <string>  //用于话题名参数

#include "nav_msgs/msg/odometry.hpp"  //Point-LIO 输出的里程计类型
#include "px4_msgs/msg/vehicle_odometry.hpp"  //PX4 DDS 外部视觉输入类型
#include "rclcpp/rclcpp.hpp"  //ROS2 C++ 节点 API

using namespace std::chrono_literals;

namespace  //匿名函数，仅在当前cpp文件中可见，避免命名冲突
{

rclcpp::QoS px4_qos()  //用在发布：/fmu/in/vehicle_visual_odometry
{
  return rclcpp::QoS(rclcpp::KeepLast(1))  //只保留最新一帧
    .best_effort()  //尽力发送，不保证可靠性
    .transient_local();  //匹配 PX4 DDS 话题常见配置
}

std::array<std::array<double, 3>, 3> quaternion_to_matrix(
  double qx, double qy, double qz, double qw)  //四元数转旋转矩阵
{  //Point-LIO 的姿态是四元数。为了完成坐标系转换，代码先把四元数转成 3x3 旋转矩阵
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);  //归一化处理
  if (norm <= 1e-9) {  //如果四元数长度异常接近 0，就返回单位矩阵，避免除 0
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
  }};  //正常四元数转旋转矩阵的计算结果
}

std::array<std::array<double, 3>, 3> matrix_multiply(  //矩阵乘法，用于计算坐标系变换
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
  return out;  //把 ROS 的姿态表达转换成 PX4 需要的姿态表达
}

std::array<float, 4> matrix_to_quaternion_wxyz(  //旋转矩阵转 PX4 四元数
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
  }};  //返回 PX4 四元数，顺序为 w, x, y, z
}  //而 ROS 消息里的 orientation 顺序为x, y, z, w

std::array<float, 4> orientation_enu_flu_to_ned_frd(  //姿态坐标系转换
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
/*
    ROS/Point-LIO 通常用：
    世界坐标：ENU
    机体坐标：FLU
    PX4 外部视觉需要：
    世界坐标：NED
    机体坐标：FRD
    含义分别是：ENU = East, North, Up  NED = North, East, Down
    FLU = Forward, Left, Up  FRD = Forward, Right, Down
    所以位置很好转：x_ned = y_enu  y_ned = x_enu  z_ned = -z_enu
    姿态不能简单交换数字，需要做旋转矩阵变换：
    r_ned_frd = enu_to_ned * r_enu_flu * frd_to_flu;
    最后再转回 PX4 四元数
*/



bool finite3(const std::array<float, 3> & values)  //检查三维向量是否都是有限数，finite3 检查
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) && std::isfinite(values[2]);
}

}  // namespace

class LocationBridge final : public rclcpp::Node  //LocationBridge 类
{
public:
  LocationBridge()
  : Node("location_bridge")  //ROS2 节点类 负责打印日志：[location_bridge]: LocationBridge started...
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/aft_mapped_to_init");  //默认订阅Point-LIO
    output_topic_ = declare_parameter<std::string>(
      "visual_odometry_topic", "/fmu/in/vehicle_visual_odometry");  //默认发布给 PX4

    origin_x_ = declare_parameter<double>("origin_x", 0.0);  //默认原点坐标为 (0, 0, 0)，可以在 launch 文件中设置
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    origin_z_ = declare_parameter<double>("origin_z", 0.0);

    zero_initial_xy_ = declare_parameter<bool>("zero_initial_xy", true);  //默认把初始位置置零
    zero_initial_z_ = declare_parameter<bool>("zero_initial_z", true);

    const double publish_rate = declare_parameter<double>("publish_rate", 30.0);  //默认发布频率为 30Hz，可以在 launch 文件中设置
    odometry_timeout_ = declare_parameter<double>("odometry_timeout", 0.5);  //默认里程计超时时间为 0.5 秒，可以在 launch 文件中设置

    position_variance_ = declare_parameter<double>("position_variance", 0.03);  //默认位置方差为 0.03，可以在 launch 文件中设置
    orientation_variance_ = declare_parameter<double>("orientation_variance", 0.05);  //默认姿态方差为 0.05，可以在 launch 文件中设置
    velocity_variance_ = declare_parameter<double>("velocity_variance", 0.05);  //默认速度方差为 0.05，可以在 launch 文件中设置

    odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(output_topic_, px4_qos());
  //创建发布器，发布 PX4 外部视觉里程计消息，使用自定义 QoS，发布到/fmu/in/vehicle_visual_odometry
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(  //订阅 Point-LIO 输出的里程计消息，使用默认 QoS
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&LocationBridge::odometry_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_rate, 1.0));
    republish_timer_ = create_wall_timer(  //创建定时器，周期为 1/publish_rate 秒，默认30HZ
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
  {  //PX4 消息时间戳单位是微秒，所以这里把 ROS2 的纳秒时间转成微秒
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }  //返回当前时间的微秒数，get_clock()->now() 返回当前时间点，.nanoseconds() 返回纳秒数，除以 1000 转换为微秒

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {  //ROS2 订阅回调函数，先取 Point-LIO 原始位置
    const double raw_x = msg->pose.pose.position.x;
    const double raw_y = msg->pose.pose.position.y;
    const double raw_z = msg->pose.pose.position.z;

    if (!xy_offset_ready_) {  //第一帧的 x/y 会被当作 0 点，如果zero_initial_z_ = true
      offset_x_ = origin_x_;
      offset_y_ = origin_y_;

      if (zero_initial_xy_) {  //第一帧 z 会被当作地面高度，如果zero_initial_z_ = true
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
    const double z_enu = raw_z + offset_z_;  //这是修正后的 ROS 本地坐标

    const auto & q = msg->pose.pose.orientation;
    const auto & twist = msg->twist.twist;

    px4_msgs::msg::VehicleOdometry odom{};
    odom.timestamp = now_us();  //PX4 消息时间戳单位是微秒，所以这里把 ROS2 的纳秒时间转成微秒
    odom.timestamp_sample = odom.timestamp;  //创建 PX4 外部视觉消息，并设置时间戳

    odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    odom.position = {{
      static_cast<float>(y_enu),  //ROS ENU -> PX4 NED
      static_cast<float>(x_enu),  //[x, y, z] -> [y, x, -z]
      static_cast<float>(-z_enu),
    }};

    odom.q = orientation_enu_flu_to_ned_frd(q.x, q.y, q.z, q.w);  //姿态转换

    odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    odom.velocity = {{  //速度转换
      static_cast<float>(twist.linear.y),
      static_cast<float>(twist.linear.x),
      static_cast<float>(-twist.linear.z),
    }};

    odom.angular_velocity = {{  //角速度转换
      static_cast<float>(twist.angular.x),
      static_cast<float>(-twist.angular.y),
      static_cast<float>(-twist.angular.z),
    }};

    odom.position_variance = {{  //协方差和质量，告诉 PX4 EKF：这份外部视觉数据大概有多可信
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
  //后续实飞时，如果 PX4 过度相信雷达定位、位置抖动明显，可以把 variance 调大一点；如果融合很慢，可以适当调小
    odom.reset_counter = 0;
    odom.quality = 100;

    if (!finite3(odom.position) || !finite3(odom.velocity)) {  //异常数据保护
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Ignoring non-finite odometry sample from %s",
        input_topic_.c_str());
      return;  //如果位置或速度有非有限数，就忽略这帧数据，避免 PX4 EKF 崩溃
    }

    latest_odom_ = odom;
    have_latest_odom_ = true;
    last_odom_time_ = now();
  //每收到 Point-LIO 新帧，会立即发布一次，同时保存起来给定时器重发
    odom_pub_->publish(odom);
  }  //订阅回调函数结束

  void republish_latest()  //定时器回调函数，定时重发最新的里程计数据
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

int main(int argc, char ** argv)  //ROS2 C++ 节点标准入口
{
  rclcpp::init(argc, argv);  //初始化 ROS2
  rclcpp::spin(std::make_shared<LocationBridge>());  //创建 LocationBridge 节点并进入循环，等待回调函数处理消息
  rclcpp::shutdown();  //关闭 ROS2
  return 0;
}
