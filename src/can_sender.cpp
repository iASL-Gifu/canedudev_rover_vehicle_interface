#include <canedudev_interface/can_sender.hpp>

namespace canedudev_interface
{
ControlCommand::ControlCommand(): Node("canedudev_interface")
{
  loop_rate_ = declare_parameter("loop_rate",800.0);
  lipo_cells = declare_parameter("lipo_cells", 4);
  if (lipo_cells == 4)
    max_velocity_ = 30.0;
  else if (lipo_cells == 3)
    max_velocity_ = 20.0;
  else
    max_velocity_ = 10.0;
  //Subscription
  actuation_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
    "/control/command/control_cmd", 10, std::bind(&canedudev_interface::ControlCommand::actuation_callback, this, std::placeholders::_1));
  gear_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 10, std::bind(&canedudev_interface::ControlCommand::gear_cmd_callback, this, std::placeholders::_1));
  //Service
  control_mode_server_ = create_service<autoware_vehicle_msgs::srv::ControlModeCommand>(
    "/control/command/control_mode_cmd", std::bind(&canedudev_interface::ControlCommand::onControlModeRequest, this, std::placeholders::_1, std::placeholders::_2));
  
  //Publisher
  can_frame_pub_ = create_publisher<can_msgs::msg::Frame>("/output/can_tx", rclcpp::QoS(1));
  control_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/control/report/control_mode_report", rclcpp::QoS(1));
  timer_ = create_timer(this, get_clock(), rclcpp::Rate(loop_rate_).period(), std::bind(&canedudev_interface::ControlCommand::timer_callback, this));
}

void ControlCommand::onControlModeRequest(
  const autoware_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
  const autoware_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
{
  // RCLCPP_INFO(get_logger(), "Received control mode command: %d", request->mode);
  switch (request->mode)
  {
    case autoware_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS:
      engage_cmd_ = true;
      response->success = true;
      return;
    case autoware_vehicle_msgs::srv::ControlModeCommand::Request::MANUAL:
      engage_cmd_ = false;
      response->success = true;
      return;
    default:
      engage_cmd_ = false;
      response->success = false;
      RCLCPP_ERROR(get_logger(), "Invalid control mode command.");
      return;
  }

}

void ControlCommand::gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
  // RCLCPP_INFO(get_logger(), "Received gear command");
  switch (msg->command)
  {
    case 2://DRIVE
      is_drive_ = true;
      is_reverse_ = false;
      break;
    case 20://REVERSE
      is_drive_ = false;
      is_reverse_ = true;
      break;
    default:
      is_drive_ = false;
      is_reverse_ = false;
      break;
  }
}

void ControlCommand::actuation_callback(const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  // if(!is_engage_)
  //   return;
  // RCLCPP_INFO(get_logger(), "Received actuation command");
  // Steering
  if (-45.0 < msg->lateral.steering_tire_angle*57.29 && msg->lateral.steering_tire_angle*57.29 < 45.0){
    steer_cmd_ = (float)msg->lateral.steering_tire_angle*57.29; //all float
    RCLCPP_INFO(get_logger(), "Steering angle: %f", steer_cmd_);
  }
  uint32_t steer_cmd_bit_;
  std::memcpy(&steer_cmd_bit_, &steer_cmd_, sizeof(float));

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
  
  // Throttle
  if (is_drive_ == true){
    //Accel

    throttle_cmd_ = (int16_t)(500.0 * ((msg->longitudinal.velocity * 3.3)/max_velocity_)  + 1550); // msg->longitudinal.velocity: m/s
    RCLCPP_INFO(get_logger(), "Throttle: %d, msg_velo %f", throttle_cmd_, msg->longitudinal.velocity);

    // if (msg->brake == 0.0)
    //   throttle_cmd_ = (int16_t)msg->longitudinal.velocity/8.0  + 1500;
    // else
    //   throttle_cmd_ = 1500 - (int16_t)msg->longitudinal.velocity /8.0* 500;
  }
  // else if(is_reverse_ == true){
  //   if (0 < msg->actuation.accel_cmd && msg->actuation.accel_cmd < 500)
  //     throttle_cmd_ = 1500 - (int16_t)msg->actuation.brake_cmd;
  //   else if (0 < msg->actuation.brake_cmd && msg->actuation.brake_cmd < 500)
  //     throttle_cmd_ = (int16_t)msg->actuation.accel_cmd + 1500;
  // }
  else{
    throttle_cmd_ = 1500;
  }
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


void ControlCommand::timer_callback()
{
  // RCLCPP_INFO(get_logger(), "Timer callback");
  if(!is_engage_){
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = get_clock()->now();
    control_mode_report.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
    control_mode_report_pub_->publish(control_mode_report);
    return;
  }
  can_frame_pub_->publish(*steer_ctrl_can_ptr_);
  can_frame_pub_->publish(*throttle_ctrl_can_ptr_);
  autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
  control_mode_report.stamp = get_clock()->now();
  control_mode_report.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  control_mode_report_pub_->publish(control_mode_report);
}

float ControlCommand::steer_bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint32_t value = (static_cast<uint32_t>(b3) << 24) |
                     (static_cast<uint32_t>(b2) << 16) |
                     (static_cast<uint32_t>(b1) <<  8) |
                     (static_cast<uint32_t>(b0));
    float result;
    std::memcpy(&result, &value, sizeof(float));
    return result;
}
}//namespace


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto sender_node  = std::make_shared<canedudev_interface::ControlCommand>();
  rclcpp::spin(sender_node);
  rclcpp::shutdown();
  return 0;
}
