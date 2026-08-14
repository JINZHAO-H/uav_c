#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>  //保存多个航点

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>  //读取mission.yaml文件

#include "uav_control_cpp/flight_bridge.hpp"  //飞控桥接类，调用 FlightBridge

using namespace std::chrono_literals;


struct MissionPoint  //把 YAML 里的一个航点变成 C++ 结构
{
  std::string name;
  std::array<float, 3> xyz{};
  double speed{0.3};
  double hold{0.0};
};

class MissionRunner final : public rclcpp::Node  //不直接发 PX4 消息，而是调用 FlightBridge
{
public:
  explicit MissionRunner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mission_runner", options)
  {
    mission_config_ = declare_parameter<std::string>(
      "mission_config", "/home/jzh/uav_c/config/mission.yaml");  //mission.yaml 文件路径
    route_ = declare_parameter<std::string>("route", "custom_points");

    rclcpp::NodeOptions bridge_options;
    bridge_options.use_global_arguments(false);
    bridge_options.parameter_overrides({
      rclcpp::Parameter("enable_offboard_publish", true),
      rclcpp::Parameter("enable_setpoint_publish", true),
      rclcpp::Parameter("request_offboard_mode", false)
    });
    bridge_ = std::make_shared<FlightBridge>(bridge_options);
  }

  void run();
  std::shared_ptr<FlightBridge> bridge() const {return bridge_;}

private:
  void load_config();
  double resolve_value(const YAML::Node & value) const;
  double resolve_speed(const YAML::Node & value) const;
  std::array<float, 3> resolve_xyz(const YAML::Node & xyz_node) const;
  std::vector<MissionPoint> load_points() const;
  void wait_armed_and_offboard(const std::array<float, 3> & initial);

  std::string mission_config_;
  std::string route_;
  YAML::Node config_;

  double origin_x_{0.0};
  double origin_y_{0.0};

  double pose_wait_timeout_{30.0};  //等待无人机到达目标点的超时时间
  double pose_timeout_{1.0};
  bool check_start_xy_{true};
  bool check_start_z_{true};
  double max_start_xy_{2.0};
  double max_start_z_{1.0};
  double offboard_arm_timeout_{25.0};  //等待无人机进入 offboard 模式并解锁的超时时间

  double default_point_speed_{0.3};
  double cruise_speed_{0.3};
  double vertical_speed_{0.45};

  std::shared_ptr<FlightBridge> bridge_;
  std::vector<MissionPoint> points_;
};


void MissionRunner::load_config()  //读取 mission.yaml，把配置变成 C++ 成员变量
{
  config_ = YAML::LoadFile(mission_config_);

  const auto mission = config_["mission"];
  if (mission && mission["route"] && route_.empty()) {
    route_ = mission["route"].as<std::string>();
  }

  const auto origin = config_["origin"];
  if (origin) {
    origin_x_ = origin["origin_x"] ? origin["origin_x"].as<double>() : 0.0;
    origin_y_ = origin["origin_y"] ? origin["origin_y"].as<double>() : 0.0;
  }

  const auto flight = config_["flight"];
  if (flight) {
    pose_wait_timeout_ = flight["pose_wait_timeout"] ?
      flight["pose_wait_timeout"].as<double>() : pose_wait_timeout_;
    pose_timeout_ = flight["pose_timeout"] ?
      flight["pose_timeout"].as<double>() : pose_timeout_;
    check_start_xy_ = flight["check_start_xy_before_arm"] ?
      flight["check_start_xy_before_arm"].as<bool>() : check_start_xy_;
    check_start_z_ = flight["check_start_z_before_arm"] ?
      flight["check_start_z_before_arm"].as<bool>() : check_start_z_;
    max_start_xy_ = flight["max_start_xy_before_arm"] ?
      flight["max_start_xy_before_arm"].as<double>() : max_start_xy_;
    max_start_z_ = flight["max_start_z_before_arm"] ?
      flight["max_start_z_before_arm"].as<double>() : max_start_z_;
    offboard_arm_timeout_ = flight["offboard_arm_timeout"] ?
      flight["offboard_arm_timeout"].as<double>() : offboard_arm_timeout_;

    default_point_speed_ = flight["default_point_speed"] ?
      flight["default_point_speed"].as<double>() : default_point_speed_;
    cruise_speed_ = flight["cruise_speed"] ?
      flight["cruise_speed"].as<double>() : default_point_speed_;
    vertical_speed_ = flight["vertical_speed"] ?
      flight["vertical_speed"].as<double>() : vertical_speed_;
  }

  points_ = load_points();

  if (points_.empty()) {
    throw std::runtime_error("Route " + route_ + " has no points");
  }

  RCLCPP_INFO(
    get_logger(),
    "Loaded mission config: %s, route=%s, points=%zu",
    mission_config_.c_str(),
    route_.c_str(),
    points_.size());
}


double MissionRunner::resolve_value(const YAML::Node & value) const
{  //解析航点坐标值，支持直接写数值，也支持写 origin_x、origin_y
  if (!value) {
    throw std::runtime_error("Missing waypoint coordinate value");
  }

  if (value.IsScalar()) {
    const std::string text = value.as<std::string>();
    if (text == "origin_x") {
      return origin_x_;
    }
    if (text == "origin_y") {
      return origin_y_;
    }
    return value.as<double>();
  }

  throw std::runtime_error("Waypoint coordinate must be a scalar");
}


double MissionRunner::resolve_speed(const YAML::Node & value) const
{  //解析航点速度值，支持直接写数值，也支持写 cruise、vertical、default
  if (!value) {
    return default_point_speed_;
  }

  if (value.IsScalar()) {
    const std::string text = value.as<std::string>();

    if (text == "cruise") {  //巡航速度
      return cruise_speed_;
    }
    if (text == "vertical") {  //垂直速度
      return vertical_speed_;
    }
    if (text == "default") {  //默认速度
      return default_point_speed_;
    }

    return value.as<double>();
  }

  throw std::runtime_error("Waypoint speed must be a scalar");
}


std::array<float, 3> MissionRunner::resolve_xyz(const YAML::Node & xyz_node) const
{  //解析航点坐标值，支持直接写数值，也支持写 origin_x、origin_y
  if (!xyz_node || !xyz_node.IsSequence() || xyz_node.size() != 3) {
    throw std::runtime_error("Waypoint xyz must be a sequence with 3 values");
  }

  return {{
    static_cast<float>(resolve_value(xyz_node[0])),
    static_cast<float>(resolve_value(xyz_node[1])),
    static_cast<float>(resolve_value(xyz_node[2])),
  }};
}


std::vector<MissionPoint> MissionRunner::load_points() const
{  //解析 mission.yaml 中的航点列表，返回 MissionPoint 的 vector
  const auto routes = config_["routes"];
  if (!routes || !routes[route_]) {
    throw std::runtime_error("Unknown route: " + route_);
  }

  const auto route = routes[route_];
  const auto points_node = route["points"] ? route["points"] : route["waypoints"];

  if (!points_node || !points_node.IsSequence()) {
    throw std::runtime_error("Route " + route_ + " has no points/waypoints list");
  }

  std::vector<MissionPoint> points;
  points.reserve(points_node.size());

  for (std::size_t i = 0; i < points_node.size(); ++i) {
    const auto item = points_node[i];

    MissionPoint point;
    point.name = item["name"] ? item["name"].as<std::string>() :
      "point_" + std::to_string(i + 1);
    point.xyz = resolve_xyz(item["xyz"]);
    point.speed = resolve_speed(item["speed"]);
    point.hold = item["hold"] ? item["hold"].as<double>() : 0.0;

    points.push_back(point);
  }

  return points;
}


void MissionRunner::wait_armed_and_offboard(const std::array<float, 3> & initial)
{  //等待无人机进入 offboard 模式并解锁 自动ARM，不主动切OFFBOARD模式，等待飞控切换
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(offboard_arm_timeout_);

  auto last_log_at = std::chrono::steady_clock::time_point{};

  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto state = bridge_->current_state();

    bridge_->set_target(initial);

    if (!state.armed) {
      bridge_->arm();
    } else if (state.mode == "OFFBOARD") {
      RCLCPP_INFO(get_logger(), "Vehicle is armed and in OFFBOARD");
      return;
    } else {
      const auto now = std::chrono::steady_clock::now();
      if (last_log_at.time_since_epoch().count() == 0 || now - last_log_at >= 2s) {
        last_log_at = now;
        RCLCPP_INFO(
          get_logger(),
          "Vehicle armed in %s; waiting for pilot to switch OFFBOARD",
          state.mode.c_str());
      }
    }

    std::this_thread::sleep_for(200ms);
  }

  throw std::runtime_error("Timed out waiting for armed OFFBOARD");
}


void MissionRunner::run()
{
  load_config();

  RCLCPP_INFO(get_logger(), "Waiting for PX4 DDS connection");
  if (!bridge_->wait_connected(20.0)) {
    throw std::runtime_error("PX4 DDS is not connected");
  }

  RCLCPP_INFO(get_logger(), "Waiting for sane PX4 local position");
  const auto [pose_ready, pose_detail] = bridge_->wait_for_sane_pose(
    pose_wait_timeout_,
    check_start_xy_,
    max_start_xy_,
    check_start_z_,
    max_start_z_,
    pose_timeout_);

  if (!pose_ready) {
    throw std::runtime_error("PX4 local position is not sane: " + pose_detail);
  }

  const auto initial = bridge_->current_pose_tuple();

  RCLCPP_INFO(get_logger(), "PX4 preflight diagnostics before ARM");
  bridge_->log_preflight_diagnostics();

  RCLCPP_INFO(get_logger(), "Priming setpoints before ARM");
  bridge_->prime_setpoints(initial);

  RCLCPP_INFO(get_logger(), "Arming and waiting for pilot-selected OFFBOARD");
  wait_armed_and_offboard(initial);

  for (const auto & point : points_) {
    RCLCPP_INFO(
      get_logger(),
      "Mission point %s: ENU(%.2f, %.2f, %.2f), speed=%.2f, hold=%.1f",
      point.name.c_str(),
      point.xyz[0],
      point.xyz[1],
      point.xyz[2],
      point.speed,
      point.hold);

    const bool reached = bridge_->goto_point(point.xyz, point.speed, point.hold, point.name);
    if (!reached) {
      throw std::runtime_error("Failed to reach mission point: " + point.name);
    }
  }

  RCLCPP_INFO(get_logger(), "Mission points completed; requesting PX4 land");
  bridge_->land();
}


int main(int argc, char ** argv)  //主函数，初始化 ROS2 节点，创建 MissionRunner 对象，运行任务
{
  rclcpp::init(argc, argv);

  auto runner = std::make_shared<MissionRunner>();  //多线程执行器

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(runner);
  executor.add_node(runner->bridge());

  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  try {
    runner->run();
    RCLCPP_INFO(runner->get_logger(), "Mission completed");
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(runner->get_logger(), "Mission failed: %s", exc.what());
    if (runner->bridge()) {
      runner->bridge()->log_preflight_diagnostics();
    }
  }

  executor.cancel();

  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
