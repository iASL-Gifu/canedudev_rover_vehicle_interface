#include <cmath>
#include "canedudev_rover_vehicle_interface/vehicle_state_receiver.hpp"

VehicleStateReceiver::VehicleStateReceiver()
  : Node("vehicle_state_receiver"),
    velocity_(0.0),
    steering_(0.0),
    battery_(0.0),
    battery_alert_(false)
{
  // Initialize parameters
  this->declare_parameter("loop_rate", 10.0);
  this->declare_parameter("velocity_threshold", 0.1);
  this->declare_parameter("battery_cell_count", 4);

  loop_rate_ = this->get_parameter("loop_rate").as_double();
  velocity_threshold_ = this->get_parameter("velocity_threshold").as_double();
  battery_cell_count_ = this->get_parameter("battery_cell_count").as_int();

  // Initialize publishers
  velocity_pub_ = this->create_publisher<VelocityReport>("/vehicle/status/velocity_status", 10);
  steering_pub_ = this->create_publisher<SteeringReport>("/vehicle/status/steering_status", 10);
  gear_pub_ = this->create_publisher<GearReport>("/vehicle/status/gear_status", 10);
  battery_pub_ = this->create_publisher<BatteryStatus>("/vehicle/status/battery_charge", 10);
  battery_alert_pub_ = this->create_publisher<std_msgs::msg::Bool>("/battery_alert", 10);

  // Initialize subscribers
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/rover/velocity", 10,
    std::bind(&VehicleStateReceiver::velocity_callback, this, std::placeholders::_1));

  steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/rover/steering", 10,
    std::bind(&VehicleStateReceiver::steering_callback, this, std::placeholders::_1));

  battery_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
    "/rover/battery_monitor_control_system/battery_output/voltage_mV", 10,
    std::bind(&VehicleStateReceiver::battery_callback, this, std::placeholders::_1));

  // Initialize timer
  state_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / loop_rate_),
    std::bind(&VehicleStateReceiver::state_timer_callback, this));
}

void VehicleStateReceiver::state_timer_callback()
{
  // Velocity
  double velocity_value = velocity_;
  auto velocity_msgs = VelocityReport();
  velocity_msgs.header.stamp = this->get_clock()->now();
  velocity_msgs.header.frame_id = "base_link";
  velocity_msgs.longitudinal_velocity = velocity_value;
  velocity_msgs.lateral_velocity = 0.0;
  velocity_msgs.heading_rate = 0.0;
  velocity_pub_->publish(velocity_msgs);

  // Steering
  double steering_value = steering_;
  auto steering_msgs = SteeringReport();
  steering_msgs.stamp = this->get_clock()->now();
  steering_msgs.steering_tire_angle = steering_value;
  steering_pub_->publish(steering_msgs);

  // Gear
  auto gear_msgs = GearReport();
  gear_msgs.stamp = this->get_clock()->now();

  if (velocity_value == 0.0 && steering_value == 0.0) {
    gear_msgs.report = GearReport::PARK;
  } else {
    gear_msgs.report = GearReport::DRIVE;
  }
  gear_pub_->publish(gear_msgs);

  // Battery
  auto battery_msgs = BatteryStatus();
  battery_msgs.stamp = this->get_clock()->now();
  battery_msgs.energy_level = battery_;
  battery_pub_->publish(battery_msgs);

  // Battery alert
  auto battery_alert_msgs = std_msgs::msg::Bool();
  battery_alert_msgs.data = battery_alert_;
  battery_alert_pub_->publish(battery_alert_msgs);
}

void VehicleStateReceiver::velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  double longitudinal_velocity = msg->vector.x;
  velocity_ = (std::abs(longitudinal_velocity) > velocity_threshold_) ? longitudinal_velocity : 0.0;
}

void VehicleStateReceiver::steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Convert steering angle from degrees to radians
  steering_ = msg->data * M_PI / 180.0;
}

void VehicleStateReceiver::battery_callback(const std_msgs::msg::UInt32::SharedPtr msg)
{
  float voltage = msg->data / 1000.0; // Convert mV to V
  float cell_voltage = voltage / battery_cell_count_;

  battery_ = estimateBatteryPercentage(cell_voltage);
  battery_alert_ = (cell_voltage < 3.50f);
}

float VehicleStateReceiver::estimateBatteryPercentage(float cell_voltage)
{
  if (cell_voltage >= 4.20f) return 100.0f;
  if (cell_voltage >= 4.00f) return interpolate(cell_voltage, 4.00f, 4.20f, 85.0f, 100.0f);
  if (cell_voltage >= 3.90f) return interpolate(cell_voltage, 3.90f, 4.00f, 70.0f, 85.0f);
  if (cell_voltage >= 3.80f) return interpolate(cell_voltage, 3.80f, 3.90f, 55.0f, 70.0f);
  if (cell_voltage >= 3.70f) return interpolate(cell_voltage, 3.70f, 3.80f, 40.0f, 55.0f);
  if (cell_voltage >= 3.60f) return interpolate(cell_voltage, 3.60f, 3.70f, 25.0f, 40.0f);
  if (cell_voltage >= 3.50f) return interpolate(cell_voltage, 3.50f, 3.60f, 10.0f, 25.0f);
  if (cell_voltage >= 3.40f) return interpolate(cell_voltage, 3.40f, 3.50f, 5.0f, 10.0f);
  if (cell_voltage >= 3.30f) return interpolate(cell_voltage, 3.30f, 3.40f, 0.0f, 5.0f);

  return 0.0f;
}

float VehicleStateReceiver::interpolate(float x, float x0, float x1, float y0, float y1)
{
  return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleStateReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}