#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_vehicle_msgs/srv/control_mode_command.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/engage.hpp"
#include "std_msgs/msg/float32.hpp"

namespace canedudev_interface
{
class ControlCommand : public rclcpp::Node {
public:
  ControlCommand();
  ~ControlCommand();

private:
  void onControlModeRequest(const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
                            const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response);
  void gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg);
  void actuation_callback(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void engage_callback(const autoware_vehicle_msgs::msg::Engage::SharedPtr msg);
  void timer_callback();

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr actuation_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Service<autoware_vehicle_msgs::srv::ControlModeCommand>::SharedPtr control_mode_server_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float loop_rate_;
  int lipo_cells;
  float max_velocity_;
  bool is_drive_ = false;
  bool is_reverse_ = false;
  bool engage_cmd_ = false;
  bool is_engage_ = false;
  float steer_cmd_;
  float throttle_cmd_;
};

ControlCommand::ControlCommand()
: Node("canedudev_interface")
{
  loop_rate_ = declare_parameter("loop_rate", 100.0);
  lipo_cells = declare_parameter("lipo_cells", 4);
  if (lipo_cells == 4)
    max_velocity_ = 30.0;
  else if (lipo_cells == 3)
    max_velocity_ = 20.0;
  else
    max_velocity_ = 10.0;

  // Subscription
  actuation_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 1, std::bind(&ControlCommand::actuation_callback, this, std::placeholders::_1));
  gear_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&ControlCommand::gear_cmd_callback, this, std::placeholders::_1));
  engage_sub_ = create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "/api/autoware/get/engage", 1, std::bind(&ControlCommand::engage_callback, this, std::placeholders::_1));

  // Service
  control_mode_server_ = create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
    "/control/command/control_mode_cmd", std::bind(&ControlCommand::onControlModeRequest, this, std::placeholders::_1, std::placeholders::_2));

  // Publisher
  throttle_pub_ = create_publisher<std_msgs::msg::Float32>("/output/throttle", rclcpp::QoS(1));
  steering_pub_ = create_publisher<std_msgs::msg::Float32>("/output/steering", rclcpp::QoS(1));
  control_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/control/report/control_mode_report", rclcpp::QoS(1));
  timer_ = create_timer(this, get_clock(), rclcpp::Rate(loop_rate_).period(), std::bind(&ControlCommand::timer_callback, this));
}

ControlCommand::~ControlCommand() {}

void ControlCommand::onControlModeRequest(const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
                                          const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
{
  // RCLCPP_INFO(get_logger(), "Received control mode request: %d", request->mode);
  switch (request->mode) {
    case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS:
      engage_cmd_ = true;
      response->success = true;
      // RCLCPP_INFO(get_logger(), "Switched to AUTONOMOUS mode");
      return;
    case autoware_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL:
      engage_cmd_ = false;
      response->success = true;
      // RCLCPP_INFO(get_logger(), "Switched to MANUAL mode");
      return;
    default:
      engage_cmd_ = false;
      response->success = false;
      // RCLCPP_ERROR(get_logger(), "Invalid control mode command.");
      return;
  }
}

void ControlCommand::gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg) {
  // RCLCPP_INFO(get_logger(), "Received gear command: %d", msg->command);
  switch (msg->command) {
    case 2: // DRIVE
      is_drive_ = true;
      is_reverse_ = false;
      // RCLCPP_INFO(get_logger(), "Gear set to DRIVE");
      break;
    case 20: // REVERSE
      is_drive_ = false;
      is_reverse_ = true;
      // RCLCPP_INFO(get_logger(), "Gear set to REVERSE");
      break;
    default:
      is_drive_ = false;
      is_reverse_ = false;
      // RCLCPP_INFO(get_logger(), "Gear set to NEUTRAL");
      break;
  }
}

void ControlCommand::actuation_callback(const autoware_control_msgs::msg::Control::SharedPtr msg) {
  // RCLCPP_INFO(get_logger(), "Received actuation command");
  if (-45.0 < msg->lateral.steering_tire_angle * 57.29 && msg->lateral.steering_tire_angle * 57.29 < 45.0) {
    steer_cmd_ = msg->lateral.steering_tire_angle * 57.29; // in degrees
    // RCLCPP_INFO(get_logger(), "Steering angle: %f", steer_cmd_);
  }

  // if (is_drive_ == true) {
  //   // Scale 0 to 100 where 50 is neutral
  //   throttle_cmd_ = 50.0 + (msg->longitudinal.velocity * 50.0) / max_velocity_; 
  //   // RCLCPP_INFO(get_logger(), "Throttle: %f, msg_velo %f", throttle_cmd_, msg->longitudinal.velocity);
  // } else if (is_reverse_ == true) {
  //   // Scale 0 to 100 where 50 is neutral
  //   throttle_cmd_ = 50.0 - (msg->longitudinal.velocity * 50.0) / max_velocity_; 
  //   // RCLCPP_INFO(get_logger(), "Throttle: %f, msg_velo %f", throttle_cmd_, msg->longitudinal.velocity);
  // } else {
  //   throttle_cmd_ = 50.0; // Neutral
  // }
  throttle_cmd_ = 50.0 + (msg->longitudinal.acceleration * 50.0) / 2.0; 
}

void ControlCommand::engage_callback(const autoware_vehicle_msgs::msg::Engage::SharedPtr msg) {
  is_engage_ = msg->engage;
}

void ControlCommand::timer_callback() {
  // RCLCPP_INFO(get_logger(), "Timer callback triggered");
  if (!is_engage_) {
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = get_clock()->now();
    control_mode_report.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    control_mode_report_pub_->publish(control_mode_report);
    // RCLCPP_INFO(get_logger(), "Published control mode report: MANUAL");
    return;
  }
  
  // Publish throttle and steering
  std_msgs::msg::Float32 throttle_msg;
  throttle_msg.data = throttle_cmd_;
  throttle_pub_->publish(throttle_msg);
  // RCLCPP_INFO(get_logger(), "Published throttle: %f", throttle_cmd_);

  std_msgs::msg::Float32 steering_msg;
  steering_msg.data = steer_cmd_;
  steering_pub_->publish(steering_msg);
  // RCLCPP_INFO(get_logger(), "Published steering: %f", steer_cmd_);

  autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
  control_mode_report.stamp = get_clock()->now();
  control_mode_report.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  control_mode_report_pub_->publish(control_mode_report);
  // RCLCPP_INFO(get_logger(), "Published control mode report: AUTONOMOUS");
}

} // namespace canedudev_interface

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<canedudev_interface::ControlCommand>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
