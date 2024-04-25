#include <canedudev_interface/can_sender.hpp>

namespace canedudev_interface
{
ControlCommand::ControlCommand(): Node("canedudev_interface")
{
  loop_rate_ = declare_parameter("loop_rate", 50.0);
  //Subscription
  actuation_sub_ = create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
    "/control/command/actuation_cmd", 10, std::bind(&canedudev_interface::ControlCommand::actuation_callback, this, std::placeholders::_1));
  gear_cmd_sub_ = create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 10, std::bind(&canedudev_interface::ControlCommand::gear_cmd_callback, this, std::placeholders::_1));
  //Service
  control_mode_server_ = create_service<autoware_auto_vehicle_msgs::srv::ControlModeCommand>(
    "/control/command/control_mode_cmd", std::bind(&canedudev_interface::ControlCommand::onControlModeRequest, this, std::placeholders::_1, std::placeholders::_2));
  
  //Publisher
  can_frame_pub_ = create_publisher<can_msgs::msg::Frame>("/output/can_tx", rclcpp::QoS(1));
  timer_ = create_timer(this, get_clock(), rclcpp::Rate(loop_rate_).period(), std::bind(&canedudev_interface::ControlCommand::timer_callback, this));
}

void ControlCommand::onControlModeRequest(
  const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::SharedPtr request,
  const autoware_auto_vehicle_msgs::srv::ControlModeCommand::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Received control mode command: %d", request->mode);
  switch (request->mode)
  {
    case autoware_auto_vehicle_msgs::srv::ControlModeCommand::Request::AUTONOMOUS:
      engage_cmd_ = true;
      response->success = true;
      return;
    case autoware_auto_vehicle_msgs::srv::ControlModeCommand::Requestk::MANUAL:
      is_engage_ = false;
      response->success = true;
      return;
    default:
      is_engage_ = false;
      response->success = false;
      RCLCPP_ERROR(get_logger(), "Invalid control mode command.");
      return;
  }

}

void ControlCommand::gear_cmd_callback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received gear command");
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

void ControlCommand::actuation_callback(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr msg)
{
  if(!is_engage_)
    return;
  // Do something with the received message
  RCLCPP_INFO(get_logger(), "Received actuation command");
  // Steering
  if (500< msg->actuation.steer_cmd && msg->actuation.steer_cmd <2500)
    steer_cmd_ = (int16_t)msg->actuation.steer_cmd; //all float
  can_msgs::msg::Frame steer_ctrl_can_msg;
  steer_ctrl_can_msg.header.stamp = msg->header.stamp;
  steer_ctrl_can_msg.id = 0x100;
  steer_ctrl_can_msg.dlc = 5;
  steer_ctrl_can_msg.is_extended = false;
  steer_ctrl_can_msg.data[0] = 0; //0:Pulse-width steering mode, 1:Angle steering mode
  steer_ctrl_can_msg.data[1] = steer_cmd_ & 0xFF;
  steer_ctrl_can_msg.data[2] = (steer_cmd_ >> 8) & 0xFF;
  steer_ctrl_can_ptr_ = std::make_shared<can_msgs::msg::Frame>(steer_ctrl_can_msg);
  
  // Throttle
  // TO-DO: Handle msg->actuation.brake_cmd_
  if (1000 < msg->actuation.accel_cmd && msg->actuation.accel_cmd < 2000)
    throttle_cmd_ = (int16_t)msg->actuation.accel_cmd;

  can_msgs::msg::Frame throttle_ctrl_can_msg;
  throttle_ctrl_can_msg.header.stamp = msg->header.stamp;
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
  RCLCPP_INFO(get_logger(), "Timer callback");
  can_frame_pub_->publish(*steer_ctrl_can_ptr_);
  can_frame_pub_->publish(*throttle_ctrl_can_ptr_);
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
