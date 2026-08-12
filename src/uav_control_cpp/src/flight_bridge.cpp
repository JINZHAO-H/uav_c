#include "uav_control_cpp/flight_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  //初始化
  rclcpp::spin(std::make_shared<FlightBridge>());  //进入 ROS2 的事件循环
  rclcpp::shutdown();
  return 0;
}
