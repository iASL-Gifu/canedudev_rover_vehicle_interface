#include "canedudev_rover_vehicle_interface/vehicle_command_sender.hpp"

VehicleCommandSender::VehicleCommandSender()
: Node("vehicle_command_sender"),
  throttle_factor_(1.0),
  plus_(false),
  minus_(false)
{
  // Initialize parameters
  get_parameters();

  // Initialize publishers
  throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rover/throttle", 10);
  steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rover/steering", 10);
  control_mode_pub_ = this->create_publisher<ControlModeReport>("/vehicle/status/control_mode", 10);

  // Initialize subscribers
  control_sub_ = this->create_subscription<Control>(
    "/control/command/control_cmd", 10,
    std::bind(&VehicleCommandSender::autoware_control_callback, this, std::placeholders::_1));

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&VehicleCommandSender::joy_callback, this, std::placeholders::_1));

  // Initialize timer
  control_mode_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&VehicleCommandSender::control_mode_timer_callback, this));
}

// !!! 速度ベースか制御ベースかは要検討 !!!
void VehicleCommandSender::autoware_control_callback(const Control::SharedPtr msg)
{
}

void VehicleCommandSender::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[throttle_plus_button_] == 1) plus_ = true;
  if (msg->buttons[throttle_minus_button_] == 1) minus_ = true;

  if (msg->buttons[throttle_plus_button_] == 1 && plus_) {
    throttle_factor_ += 0.1;
    plus_ = false;

    RCLCPP_INFO(this->get_logger(), "Throttle factor increased to: %f", throttle_factor_);
  }

  if (msg->buttons[throttle_minus_button_] == 1 && minus_) {
    throttle_factor_ -= 0.1;
    minus_ = false;

    RCLCPP_INFO(this->get_logger(), "Throttle factor decreased to: %f", throttle_factor_);
  }

  if (throttle_factor_ > 1.0) throttle_factor_ = 1.0;
  if (throttle_factor_ < 0.0) throttle_factor_ = 0.0;

  if (msg->buttons[manual_button_] == 1) {
    manual_flag_ = true;

    double throttle_value = msg->axes[throttle_axes_];
    double steering_value = msg->axes[steering_axes_];

    double throttle = -1 * throttle_value * throttle_factor_ * throttle_limit_;
    double steering = -1 * steering_value * steering_limit_;

    send_control_command(throttle, steering);
  } else {
    manual_flag_ = false;
  }
}

void VehicleCommandSender::control_mode_timer_callback()
{
  auto control_mode_msg = ControlModeReport();
  control_mode_msg.stamp = this->get_clock()->now();
  control_mode_msg.mode = manual_flag_ ? ControlModeReport::MANUAL : ControlModeReport::AUTONOMOUS;

  control_mode_pub_->publish(control_mode_msg);
}

void VehicleCommandSender::send_control_command(double throttle, double steering)
{
  auto throttle_msg = std_msgs::msg::Float32();
  auto steering_msg = std_msgs::msg::Float32();

  throttle_msg.data = throttle;
  steering_msg.data = steering;

  throttle_pub_->publish(throttle_msg);
  steering_pub_->publish(steering_msg);
}

void VehicleCommandSender::get_parameters()
{
  this->declare_parameter("throttle_limit", 1.0);
  this->declare_parameter("steering_limit", 1.0);

  throttle_limit_ = this->get_parameter("throttle_limit").as_double();
  steering_limit_ = this->get_parameter("steering_limit").as_double();

  this->declare_parameter("manual_button",       0);
  this->declare_parameter("throttle_plus_button",   0);
  this->declare_parameter("throttle_minus_button",  0);
  this->declare_parameter("throttle_axes",          0);
  this->declare_parameter("steering_axes",          0);

  manual_button_      = this->get_parameter("manual_button").as_int();
  throttle_plus_button_  = this->get_parameter("throttle_plus_button").as_int();
  throttle_minus_button_ = this->get_parameter("throttle_minus_button").as_int();
  throttle_axes_         = this->get_parameter("throttle_axes").as_int();
  steering_axes_         = this->get_parameter("steering_axes").as_int();

  RCLCPP_INFO(this->get_logger(), "Parameters:");
  RCLCPP_INFO(this->get_logger(), "  throttle_limit:       %f", throttle_limit_);
  RCLCPP_INFO(this->get_logger(), "  steering_limit:       %f", steering_limit_);
  RCLCPP_INFO(this->get_logger(), "  manual_button:       %d", manual_button_);
  RCLCPP_INFO(this->get_logger(), "  throttle_plus_button:   %d", throttle_plus_button_);
  RCLCPP_INFO(this->get_logger(), "  throttle_minus_button:  %d", throttle_minus_button_);
  RCLCPP_INFO(this->get_logger(), "  throttle_axes:          %d", throttle_axes_);
  RCLCPP_INFO(this->get_logger(), "  steering_axes:          %d", steering_axes_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleCommandSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
