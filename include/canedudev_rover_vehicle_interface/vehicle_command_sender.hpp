#ifndef VEHICLE_COMMAND_SENDER_HPP__
#define VEHICLE_COMMAND_SENDER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"

class VehicleCommandSender : public rclcpp::Node
{

public:
  using Control = autoware_control_msgs::msg::Control;
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;

  VehicleCommandSender();

private:
  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<ControlModeReport>::SharedPtr control_mode_pub_;

  // Subscriber
  rclcpp::Subscription<Control>::SharedPtr control_sub_;
  void autoware_control_callback(const Control::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Client

  // Server

  // Timer
  rclcpp::TimerBase::SharedPtr control_mode_timer_;
  void control_mode_timer_callback();

  // Parameter
  void get_parameters();
  double throttle_limit_, steering_limit_;
  int manual_button_;
  int throttle_plus_button_, throttle_minus_button_;
  int throttle_axes_, steering_axes_;

  // function
  void send_control_command(double throttle, double steering);

  // variable
  double throttle_factor_;
  bool plus_, minus_, manual_flag_;
};

#endif