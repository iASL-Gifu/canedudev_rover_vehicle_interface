#ifndef VEHICLE_STATE_RECEIVER_HPP__
#define VEHICLE_STATE_RECEIVER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"

class VehicleStateReceiver : public rclcpp::Node
{

public:
  using VelocityReport = autoware_vehicle_msgs::msg::VelocityReport;
  using SteeringReport = autoware_vehicle_msgs::msg::SteeringReport;
  using GearReport = autoware_vehicle_msgs::msg::GearReport;
  using BatteryStatus = tier4_vehicle_msgs::msg::BatteryStatus;

  VehicleStateReceiver();

private:
  // Publisher
  rclcpp::Publisher<VelocityReport>::SharedPtr velocity_pub_;
  rclcpp::Publisher<SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<GearReport>::SharedPtr gear_pub_;
  rclcpp::Publisher<BatteryStatus>::SharedPtr battery_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr battery_alert_pub_;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;
  void velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  void steering_callback(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr battery_sub_;
  void battery_callback(const std_msgs::msg::UInt32::SharedPtr msg);


  // Client

  // Server

  // Timer
  rclcpp::TimerBase::SharedPtr state_timer_;
  void state_timer_callback();

  // Parameter
  double loop_rate_;
  double velocity_threshold_;
  double battery_cell_count_;

  // function
  float estimateBatteryPercentage(float voltage);
  float interpolate(float x, float x0, float y0, float x1, float y1);

  // variable
  double velocity_, steering_, battery_;
  bool battery_alert_;
};

#endif