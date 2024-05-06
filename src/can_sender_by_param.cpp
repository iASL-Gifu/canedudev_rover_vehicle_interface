#include <canedudev_interface/can_sender_by_param.hpp>

namespace canedudev_interface
{
ControlCommandbyParam::ControlCommandbyParam(): Node("canedudev_interface")
{
  loop_rate_ = declare_parameter("loop_rate",10.0);
  //Publisher
  can_frame_pub_ = create_publisher<can_msgs::msg::Frame>("/output/can_tx", rclcpp::QoS(1));
  timer_ = create_timer(this, get_clock(), rclcpp::Rate(loop_rate_).period(), std::bind(&canedudev_interface::ControlCommandbyParam::timer_callback, this));
  steer_cmd_    = declare_parameter_with_min_max("steer_cmd", 0.0, -90.0, 90.0, "Steering command", "Steering command in degree");
  throttle_cmd_  = (uint16_t)declare_parameter_with_min_max("throttle_cmd", 1500, 0, 2000, "Throttle command", "Throttle command in pulse-width");
  set_param_res_ = add_on_set_parameters_callback(std::bind(&ControlCommandbyParam::onParameter, this, std::placeholders::_1));
}


void ControlCommandbyParam::actuation_callback()
{
  // RCLCPP_INFO(get_logger(), "Received actuation command");
  // Steering
  RCLCPP_INFO(get_logger(), "Steer_cmd: %f", steer_cmd_);
  can_msgs::msg::Frame steer_ctrl_can_msg;
  steer_ctrl_can_msg.header.stamp = this->get_clock()->now();
  steer_ctrl_can_msg.id = 0x100;
  steer_ctrl_can_msg.dlc = 5;
  steer_ctrl_can_msg.is_extended = false;
  steer_ctrl_can_msg.data[0] = 1; //0:Pulse-width steering mode, 1:Angle steering mode
  steer_ctrl_can_msg.data[1] =  steer_cmd_bit_ & 0xFF;
  steer_ctrl_can_msg.data[2] = (steer_cmd_bit_ >> 8) & 0xFF;
  steer_ctrl_can_msg.data[3] = (steer_cmd_bit_ >> 16) & 0xFF;
  steer_ctrl_can_msg.data[4] = (steer_cmd_bit_ >> 24) & 0xFF;
  
  steer_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(steer_ctrl_can_msg);
  
  //throttle
  RCLCPP_INFO(get_logger(), "Throttle_cmd: %d", throttle_cmd_);
  can_msgs::msg::Frame throttle_ctrl_can_msg;
  throttle_ctrl_can_msg.header.stamp = this->get_clock()->now();
  throttle_ctrl_can_msg.id = 0x101;
  throttle_ctrl_can_msg.dlc = 5;
  throttle_ctrl_can_msg.is_extended = false;
  throttle_ctrl_can_msg.data[0] = 0;
  throttle_ctrl_can_msg.data[1] = throttle_cmd_ & 0xFF;
  throttle_ctrl_can_msg.data[2] = (throttle_cmd_ >> 8) & 0xFF;
  throttle_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(throttle_ctrl_can_msg);

}


void ControlCommandbyParam::timer_callback()
{
  // RCLCPP_INFO(get_logger(), "Timer callback");
  actuation_callback();
  
  can_frame_pub_->publish(*steer_ctrl_can_ptr_);
  can_frame_pub_->publish(*throttle_ctrl_can_ptr_);
}

float ControlCommandbyParam::steer_bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint32_t value = (static_cast<uint32_t>(b3) << 24) |
                     (static_cast<uint32_t>(b2) << 16) |
                     (static_cast<uint32_t>(b1) <<  8) |
                     (static_cast<uint32_t>(b0));
    float result;
    std::memcpy(&result, &value, sizeof(float));
    return result;
}

float ControlCommandbyParam::declare_parameter_with_min_max(
  const std::string & name, const float default_value, const float min_value,
  const float max_value, const std::string & description_name, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.name = description_name;
  desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  desc.description = description;
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = min_value;
  range.to_value = max_value;
  desc.floating_point_range.push_back(range);
  return this->declare_parameter(name, default_value, desc);
}

SetParametersResult ControlCommandbyParam::onParameter(const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_INFO(get_logger(), "Received parameter");
  SetParametersResult result;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "steer_cmd") {
      steer_cmd_ = (float)parameter.as_double();
      result.successful = true;
    }
    else if (parameter.get_name() == "throttle_cmd") {
      throttle_cmd_ = parameter.as_double();
      result.successful = true;
    }
  }
  return result;
}
}//namespace


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto sender_node  = std::make_shared<canedudev_interface::ControlCommandbyParam>();
  rclcpp::spin(sender_node);
  rclcpp::shutdown();
  return 0;
}
