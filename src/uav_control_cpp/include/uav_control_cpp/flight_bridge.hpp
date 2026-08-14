#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "px4_msgs/msg/offboard_control_mode.hpp"  //告诉 PX4 当前用什么控制方式
#include "px4_msgs/msg/trajectory_setpoint.hpp"  //给 PX4 发位置/速度/航向目标
#include "px4_msgs/msg/vehicle_command.hpp"  //给 PX4 发 ARM、OFFBOARD、LAND 之类命令
#include "px4_msgs/msg/vehicle_local_position.hpp"  //读 PX4 当前本地位置
#include "px4_msgs/msg/vehicle_status.hpp"  //读 PX4 当前状态
#include "rclcpp/rclcpp.hpp"  //
#include "px4_msgs/msg/vehicle_command_ack.hpp"  //知道 ARM / LAND / SET_MODE 命令有没有被 PX4 接受
#include "px4_msgs/msg/event.hpp"  //读取 PX4 commander / preflight / failsafe 事件
#include "px4_msgs/msg/estimator_status_flags.hpp"  //判断 PX4 EKF 是否对齐、是否有 external vision、local position 是否可靠
#include "px4_msgs/msg/vehicle_land_detected.hpp"  //判断 PX4 认为飞机是在地上、空中、降落中

using namespace std::chrono_literals;

namespace
{

rclcpp::QoS px4_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1))  //只保留最新一帧
    .best_effort()  //尽力传，不强求每帧都到
    .transient_local();  //和 PX4 DDS 习惯匹配
}

std::array<float, 3> enu_to_ned(const std::array<float, 3> & xyz)
{  //做坐标转换
  return {{
    xyz[1],
    xyz[0],
    -xyz[2],
  }};
}  // ENU -> NED

double distance_xy(const std::array<float, 3> & a, const std::array<float, 3> & b)
{  //计算两点的水平距离
  const double dx = static_cast<double>(a[0] - b[0]);
  const double dy = static_cast<double>(a[1] - b[1]);
  return std::sqrt(dx * dx + dy * dy);
}

double distance_3d(const std::array<float, 3> & a, const std::array<float, 3> & b)
{  //计算两点的三维距离
  const double dx = static_cast<double>(a[0] - b[0]);
  const double dy = static_cast<double>(a[1] - b[1]);
  const double dz = static_cast<double>(a[2] - b[2]);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool finite3(const std::array<float, 3> & xyz)
{
  return std::isfinite(xyz[0]) && std::isfinite(xyz[1]) && std::isfinite(xyz[2]);
}

double normalize_yaw(double yaw)
{
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;  //将偏航角归一化到 [-π, π)
}

}  // namespace

struct BridgeState  //用来保存 PX4 当前状态
{
  bool connected{false};
  bool armed{false};
  bool guided{false};
  bool manual_input{false};
  std::string mode;
  int system_status{0};
};

class FlightBridge final : public rclcpp::Node
{
public:  //读参数，建发布器，建订阅器，建定时器
  explicit FlightBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("flight_bridge", options)
  {
    rate_hz_ = std::max(declare_parameter<double>("setpoint_rate", 40.0), 1.0);
    prime_seconds_ = std::max(declare_parameter<double>("prime_seconds", 2.0), 0.0);  //进入 OFFBOARD 前提前发 setpoint 的时间
    tolerance_xy_ = std::max(declare_parameter<double>("tolerance_xy", 0.1), 0.01);  //飞点的水平容忍度，单位 m
    tolerance_z_ = std::max(declare_parameter<double>("tolerance_z", 0.1), 0.01);  //飞点的垂直容忍度，单位 m
    min_timeout_ = std::max(declare_parameter<double>("min_timeout", 15.0), 1.0);  //飞点的最小超时时间，单位 s
    timeout_per_meter_ = std::max(declare_parameter<double>("timeout_per_meter", 10.0), 1.0);  //飞点的每米超时时间，单位 s/m
    preserve_current_yaw_ = declare_parameter<bool>("preserve_current_yaw", true);  //进入 OFFBOARD 后保持当前航向
    respect_external_takeover_ = declare_parameter<bool>("respect_external_takeover", true);  //是否尊重飞手/地面站接管
    request_offboard_mode_ = declare_parameter<bool>("request_offboard_mode", false);  //是否在启动时请求 OFFBOARD 模式
    target_system_ = declare_parameter<int>("target_system", 1);  //PX4 目标系统号
    target_component_ = declare_parameter<int>("target_component", 1);  //PX4 目标组件号
    source_system_ = declare_parameter<int>("source_system", 1);
    source_component_ = declare_parameter<int>("source_component", 1);
    enable_offboard_publish_ = declare_parameter<bool>("enable_offboard_publish", false);
    enable_setpoint_publish_ = declare_parameter<bool>("enable_setpoint_publish", false);

    offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", px4_qos());  //告诉 PX4 当前用什么控制方式
    setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos());  //给 PX4 发位置/速度/航向目标
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", px4_qos());  //给 PX4 发 ARM、OFFBOARD、LAND 之类命令

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(  //订阅两个 PX4 输出话题
      "/fmu/out/vehicle_status",
      px4_qos(),
      std::bind(&FlightBridge::vehicle_status_cb, this, std::placeholders::_1));

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",
      px4_qos(),
      std::bind(&FlightBridge::vehicle_local_position_cb, this, std::placeholders::_1));

    command_ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
       "/fmu/out/vehicle_command_ack",
      px4_qos(),  //命令是否被接受
      std::bind(&FlightBridge::vehicle_command_ack_cb, this, std::placeholders::_1));

    event_sub_ = create_subscription<px4_msgs::msg::Event>(
      "/fmu/out/event",
      px4_qos(),  //PX4 报了什么事件
      std::bind(&FlightBridge::event_cb, this, std::placeholders::_1));

    estimator_flags_sub_ = create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
      "/fmu/out/estimator_status_flags",
      px4_qos(),  //EKF 是否健康
      std::bind(&FlightBridge::estimator_flags_cb, this, std::placeholders::_1));

    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected",
      px4_qos(),  //PX4 是否认为飞机在地面
      std::bind(&FlightBridge::land_detected_cb, this, std::placeholders::_1));

    publish_timer_ = create_wall_timer(  //按固定频率执行 publish_once()
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_hz_)),
      std::bind(&FlightBridge::publish_once, this));

    RCLCPP_INFO(get_logger(), "FlightBridge started");
  }

  BridgeState current_state() const  //把当前 PX4 状态返回给外部
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  std::array<float, 3> current_pose_tuple() const  //把当前 PX4 本地位置返回给外部
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {{
      local_position_.y,
      local_position_.x,
      -local_position_.z,
    }};
  }

  bool wait_connected(double timeout_sec = 20.0)  //等待 PX4 连接
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::max(timeout_sec, 0.0));
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (current_state().connected) {
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    return current_state().connected;
  }

  bool has_recent_pose(double timeout_sec) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (local_position_received_at_.nanoseconds() == 0) {
      return false;
    }
    const double age = (now() - local_position_received_at_).seconds();
    return age <= timeout_sec && local_position_.xy_valid && local_position_.z_valid;
  }

  std::pair<bool, std::string> local_pose_sanity(
    bool check_xy = true,
    double max_xy = 2.0,
    bool check_z = true,
    double max_z = 1.0,
    double max_age_sec = 1.0) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (local_position_received_at_.nanoseconds() == 0) {
      return {false, "no /fmu/out/vehicle_local_position received"};
    }

    const double age = (now() - local_position_received_at_).seconds();
    if (age > max_age_sec) {
      std::ostringstream stream;
      stream << "/fmu/out/vehicle_local_position is stale (" << age << "s old)";
      return {false, stream.str()};
    }
    if (!local_position_.xy_valid) {
      return {false, "/fmu/out/vehicle_local_position xy_valid is false"};
    }
    if (!local_position_.z_valid) {
      return {false, "/fmu/out/vehicle_local_position z_valid is false"};
    }

    const std::array<float, 3> pose_enu{{
      local_position_.y,
      local_position_.x,
      -local_position_.z,
    }};
    if (!finite3(pose_enu)) {
      return {false, "local pose contains non-finite values"};
    }
    if (check_xy && std::max(std::fabs(pose_enu[0]), std::fabs(pose_enu[1])) > max_xy) {
      std::ostringstream stream;
      stream << "local pose x/y=(" << pose_enu[0] << ", " << pose_enu[1]
             << ") exceeds max_xy=" << max_xy;
      return {false, stream.str()};
    }
    if (check_z && std::fabs(pose_enu[2]) > max_z) {
      std::ostringstream stream;
      stream << "local pose z=" << pose_enu[2] << " exceeds max_z=" << max_z;
      return {false, stream.str()};
    }

    std::ostringstream stream;
    stream << "local pose sane: (" << pose_enu[0] << ", " << pose_enu[1] << ", "
           << pose_enu[2] << "), age=" << age << "s";
    return {true, stream.str()};
  }

  std::pair<bool, std::string> wait_for_sane_pose(
    double timeout_sec,
    bool check_xy = true,
    double max_xy = 2.0,
    bool check_z = true,
    double max_z = 1.0,
    double max_age_sec = 1.0)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(std::max(timeout_sec, 0.0));
    std::string last_detail = "no /fmu/out/vehicle_local_position received";
    auto last_log_at = std::chrono::steady_clock::time_point{};

    while (rclcpp::ok()) {
      const auto [ready, detail] = local_pose_sanity(
        check_xy, max_xy, check_z, max_z, max_age_sec);
      if (ready) {
        return {true, detail};
      }
      last_detail = detail;

      const auto now_steady = std::chrono::steady_clock::now();
      if (now_steady >= deadline) {
        return {false, last_detail};
      }
      if (last_log_at.time_since_epoch().count() == 0 ||
        now_steady - last_log_at >= 2s)
      {
        last_log_at = now_steady;
        RCLCPP_WARN(get_logger(), "Waiting for sane PX4 local pose: %s", detail.c_str());
      }
      std::this_thread::sleep_for(50ms);
    }

    return {false, last_detail};
  }

  void log_preflight_diagnostics() const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    RCLCPP_INFO(
      get_logger(),
      "Preflight diagnostics: connected=%d armed=%d mode=%s local_xy_valid=%d local_z_valid=%d",
      state_.connected,
      state_.armed,
      state_.mode.c_str(),
      local_position_.xy_valid,
      local_position_.z_valid);

    if (estimator_flags_received_at_.nanoseconds() != 0) {
      RCLCPP_INFO(
        get_logger(),
        "Estimator flags: tilt_align=%d yaw_align=%d ev_pos=%d ev_hgt=%d ev_yaw=%d fake_pos=%d fake_hgt=%d",
        estimator_flags_.cs_tilt_align,
        estimator_flags_.cs_yaw_align,
        estimator_flags_.cs_ev_pos,
        estimator_flags_.cs_ev_hgt,
        estimator_flags_.cs_ev_yaw,
        estimator_flags_.cs_fake_pos,
        estimator_flags_.cs_fake_hgt);
    } else {
      RCLCPP_WARN(get_logger(), "No /fmu/out/estimator_status_flags received yet");
    }

    if (command_ack_received_at_.nanoseconds() != 0) {
      RCLCPP_INFO(
        get_logger(),
        "Last command ack: command=%u result=%u result_param2=%u",
        last_command_ack_.command,
        last_command_ack_.result,
        last_command_ack_.result_param2);
    } else {
      RCLCPP_WARN(get_logger(), "No /fmu/out/vehicle_command_ack received yet");
    }

    if (event_received_at_.nanoseconds() != 0) {
      RCLCPP_INFO(
        get_logger(),
        "Last PX4 event: id=%u log_levels=%u",
        last_event_.id,
        last_event_.log_levels);
    } else {
      RCLCPP_WARN(get_logger(), "No /fmu/out/event received yet");
    }

    if (land_detected_received_at_.nanoseconds() != 0) {
      RCLCPP_INFO(
        get_logger(),
        "Land detected: landed=%d maybe_landed=%d ground_contact=%d freefall=%d",
        land_detected_.landed,
        land_detected_.maybe_landed,
        land_detected_.ground_contact,
        land_detected_.freefall);
    } else {
      RCLCPP_WARN(get_logger(), "No /fmu/out/vehicle_land_detected received yet");
    }
  }

  bool target_reached(
    const std::array<float, 3> & current_enu,
    const std::array<float, 3> & target_enu) const
  {  //判断当前 PX4 本地位置是否到达目标点
    const double xy_error = distance_xy(current_enu, target_enu);
    const double z_error = std::fabs(static_cast<double>(current_enu[2] - target_enu[2]));
    return xy_error <= tolerance_xy_ && z_error <= tolerance_z_;
  }

  void set_target(
    const std::array<float, 3> & target_enu,
    double speed = 0.0)  //飞点的入口之一
  {
    std::lock_guard<std::mutex> lock(mutex_);  //锁住 mutex_，防止多线程同时访问 target_enu_ 和 has_target_
    target_enu_ = target_enu;
    target_speed_ = std::max(speed, 0.0);
    has_target_ = true;
    if (preserve_current_yaw_) {
      target_yaw_ = current_heading_locked();
    }
    active_ = true;
  }

  void set_active(bool active)  //飞点的入口之二，控制“开/关输出”
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = active;
  }

  void hold(
    const std::array<float, 3> & target_enu,
    double seconds,
    const std::string & name = "hold")
  {  //在当前位置悬停一段时间，持续把同一个目标点写入 FlightBridge
    const auto end_time = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(std::max(seconds, 0.0));

    RCLCPP_INFO(
      get_logger(),
      "Holding %s at ENU(%.2f, %.2f, %.2f) for %.1fs",
      name.c_str(),
      target_enu[0],
      target_enu[1],
      target_enu[2],
      seconds);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
      set_target(target_enu);
      std::this_thread::sleep_for(50ms);
    }
  }

  bool goto_point(
    const std::array<float, 3> & target_enu,
    double speed,
    double hold_sec = 0.0,
    const std::string & name = "point")
  {  //飞到指定点，持续把同一个目标点写入 FlightBridge，直到到达目标点或超时
    const auto [pose_ready, pose_detail] = wait_for_sane_pose(
      5.0, false, 0.0, false, 0.0, 1.0);
    if (!pose_ready) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot fly to %s before PX4 local pose is ready: %s",
        name.c_str(),
        pose_detail.c_str());
      return false;
    }

    const auto start_pose = current_pose_tuple();
    const double distance = distance_3d(start_pose, target_enu);
    const double effective_speed = std::max(speed, 0.0);
    const double timeout_sec = effective_speed > 0.0 ?
      std::max(min_timeout_, distance / effective_speed * 3.0) :
      std::max(min_timeout_, distance * timeout_per_meter_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);

    RCLCPP_INFO(
      get_logger(),
      "Flying to %s ENU(%.2f, %.2f, %.2f), distance=%.2fm, speed=%.2fm/s, timeout=%.1fs",
      name.c_str(),
      target_enu[0],
      target_enu[1],
      target_enu[2],
      distance,
      effective_speed,
      timeout_sec);

    set_target(target_enu, effective_speed);

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      const auto current = current_pose_tuple();

      if (target_reached(current, target_enu)) {
        RCLCPP_INFO(get_logger(), "Reached %s", name.c_str());

        if (hold_sec > 0.0) {
          hold(target_enu, hold_sec, name);
        }

        return true;
      }

      std::this_thread::sleep_for(50ms);
    }

    const auto current = current_pose_tuple();
    RCLCPP_ERROR(
      get_logger(),
      "Timed out flying to %s. Current ENU(%.2f, %.2f, %.2f), target ENU(%.2f, %.2f, %.2f)",
      name.c_str(),
      current[0],
      current[1],
      current[2],
      target_enu[0],
      target_enu[1],
      target_enu[2]);

    return false;
  }

  void prime_setpoints(const std::array<float, 3> & target_enu)
  {  //PX4 进入 OFFBOARD 前，需要先连续发一段时间的 setpoint，让飞控确认“外部控制源是稳定存在的”
    set_target(target_enu);
    const auto end_time = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(prime_seconds_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end_time) {
      publish_once();
      std::this_thread::sleep_for(50ms);
    }
  }

  void arm()  //ARM 无人机
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
  }

  void disarm()  //DISARM 无人机
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
  }

  void enter_offboard()  //请求 PX4 进入 OFFBOARD 模式
  {
    if (!request_offboard_mode_) {
      return;
    }
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
  }

  void land()  //请求 PX4 降落
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  void shutdown()  //关闭飞行桥接
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    has_target_ = false;
  }

private:
  uint64_t now_us()  //把 ROS 时间转成微秒
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  double current_heading_locked() const
  {
    return static_cast<double>(local_position_.heading);
  }

  void vehicle_status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {  //PX4 状态回调函数，更新 state_ 结构体
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = true;
    state_.armed = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
    state_.guided = msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    state_.manual_input = false;
    state_.system_status = static_cast<int>(msg->nav_state);
    state_.mode = nav_state_name(msg->nav_state);
  }

  void vehicle_local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {  //PX4 本地位置回调函数，更新 local_position_ 结构体
    std::lock_guard<std::mutex> lock(mutex_);
    local_position_ = *msg;
    local_position_received_at_ = now();
  }

  void vehicle_command_ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
  {  //PX4 命令反馈回调函数，更新 last_command_ack_ 结构体
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_ack_ = *msg;
    command_ack_received_at_ = now();
  }

  void event_cb(const px4_msgs::msg::Event::SharedPtr msg)
  {  //PX4 事件回调函数，更新 last_event_ 结构体
    std::lock_guard<std::mutex> lock(mutex_);
    last_event_ = *msg;
    event_received_at_ = now();
  }

  void estimator_flags_cb(const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg)
  {  //PX4 EKF 状态回调函数，更新 estimator_flags_ 结构体
    std::lock_guard<std::mutex> lock(mutex_);
    estimator_flags_ = *msg;
    estimator_flags_received_at_ = now();
  }

  void land_detected_cb(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
  {  //PX4 降落状态回调函数，更新 land_detected_ 结构体
    std::lock_guard<std::mutex> lock(mutex_);
    land_detected_ = *msg;
    land_detected_received_at_ = now();
  }

  std::string nav_state_name(uint8_t nav_state) const
  {  //把 PX4 的 nav_state 转成字符串，方便打印
    switch (nav_state) {
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_MANUAL:
        return "MANUAL";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_ALTCTL:
        return "ALTCTL";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
        return "POSCTL";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
        return "AUTO.MISSION";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
        return "AUTO.LOITER";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
        return "AUTO.RTL";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
        return "OFFBOARD";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
        return "AUTO.TAKEOFF";
      case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
        return "AUTO.LAND";
      default:
        return "UNKNOWN";
    }
  }

  void publish_vehicle_command(uint16_t command, float p1 = 0.0F, float p2 = 0.0F)
  {  //给 PX4 发命令的通用函数，command 是命令号，p1/p2 是参数
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = now_us();
    msg.param1 = p1;
    msg.param2 = p2;
    msg.param3 = 0.0F;
    msg.param4 = 0.0F;
    msg.param5 = 0.0;
    msg.param6 = 0.0;
    msg.param7 = 0.0F;
    msg.command = command;
    msg.target_system = static_cast<uint8_t>(target_system_);
    msg.target_component = static_cast<uint8_t>(target_component_);
    msg.source_system = static_cast<uint8_t>(source_system_);
    msg.source_component = static_cast<uint16_t>(source_component_);
    msg.confirmation = 0;
    msg.from_external = true;
    command_pub_->publish(msg);
  }

  void publish_offboard_control_mode()
  {  //告诉 PX4 当前用什么控制方式，OFFBOARD 模式下必须每 1 秒至少发一次
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = now_us();
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_pub_->publish(msg);
  }

  void publish_trajectory_setpoint()
  {  //给 PX4 发位置/速度/航向目标，OFFBOARD 模式下必须每 1 秒至少发一次
    std::array<float, 3> target{};
    std::array<float, 3> current{};
    double yaw = 0.0;
    double speed = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_target_) {
        return;
      }
      target = target_enu_;
      current = {{
        local_position_.y,
        local_position_.x,
        -local_position_.z,
      }};
      yaw = target_yaw_;
      speed = target_speed_;
    }

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = now_us();
    const auto target_ned = enu_to_ned(target);
    msg.position = {{
      target_ned[0],
      target_ned[1],
      target_ned[2],
    }};
    const double distance = distance_3d(current, target);
    if (speed > 0.0 && distance > tolerance_xy_) {
      const std::array<float, 3> velocity_enu{{
        static_cast<float>((target[0] - current[0]) / distance * speed),
        static_cast<float>((target[1] - current[1]) / distance * speed),
        static_cast<float>((target[2] - current[2]) / distance * speed),
      }};
      const auto velocity_ned = enu_to_ned(velocity_enu);
      msg.velocity = {{
        velocity_ned[0],
        velocity_ned[1],
        velocity_ned[2],
      }};
    } else {
      msg.velocity = {{
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::quiet_NaN(),
      }};
    }
    msg.acceleration = {{
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
    }};
    msg.jerk = {{
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
    }};
    msg.yaw = static_cast<float>(normalize_yaw(yaw));
    msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
    setpoint_pub_->publish(msg);
  }

  void publish_once()  //发布一次所有消息
  {
    bool publish_offboard = false;
    bool publish_setpoint = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) {
        return;
      }
      if (respect_external_takeover_ && !state_.armed && state_.connected) {
        active_ = false;
        return;
      }
      publish_offboard = enable_offboard_publish_;
      publish_setpoint = enable_setpoint_publish_;
    }

    if (publish_offboard) {
      publish_offboard_control_mode();
    }
    if (publish_setpoint) {
      publish_trajectory_setpoint();
    }
  }

  mutable std::mutex mutex_;
  BridgeState state_{};
  px4_msgs::msg::VehicleLocalPosition local_position_{};
  rclcpp::Time local_position_received_at_{0, 0, RCL_ROS_TIME};

  px4_msgs::msg::VehicleCommandAck last_command_ack_{};
  rclcpp::Time command_ack_received_at_{0, 0, RCL_ROS_TIME};

  px4_msgs::msg::Event last_event_{};
  rclcpp::Time event_received_at_{0, 0, RCL_ROS_TIME};

  px4_msgs::msg::EstimatorStatusFlags estimator_flags_{};
  rclcpp::Time estimator_flags_received_at_{0, 0, RCL_ROS_TIME};

  px4_msgs::msg::VehicleLandDetected land_detected_{};
  rclcpp::Time land_detected_received_at_{0, 0, RCL_ROS_TIME};

  std::array<float, 3> target_enu_{};
  double target_yaw_{0.0};
  double target_speed_{0.0};
  bool has_target_{false};
  bool active_{false};

  double rate_hz_{40.0};
  double prime_seconds_{2.0};
  double tolerance_xy_{0.1};
  double tolerance_z_{0.1};
  double min_timeout_{15.0};
  double timeout_per_meter_{10.0};
  bool preserve_current_yaw_{true};
  bool respect_external_takeover_{true};
  bool request_offboard_mode_{false};
  bool enable_offboard_publish_{false};
  bool enable_setpoint_publish_{false};
  int target_system_{1};
  int target_component_{1};
  int source_system_{1};
  int source_component_{1};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_sub_;
  rclcpp::Subscription<px4_msgs::msg::Event>::SharedPtr event_sub_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr estimator_flags_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
