#include <canedudev_interface/can_receiver.hpp>
namespace canedudev_interface
{
VehicleReport::VehicleReport(): Node("vehicle_report_node")
{
    frame_id_  = declare_parameter("frame_id", "base_link");
    loop_rate_ = declare_parameter("loop_rate", 50.0);
    //Subscription
    can_frame_sub_ = create_subscription<can_msgs::msg::Frame>(
        "/input/can_rx", 10, std::bind(&canedudev_interface::VehicleReport::can_frame_callback, this, std::placeholders::_1));
    steering_report_pub_ = create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/steering_report", rclcpp::QoS(1));
    throttle_report_pub_ = create_publisher<std_msgs::msg::UInt16>("/vehicle/throttle_report", rclcpp::QoS(1));
    battery_report_pub_  = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/battery_report", rclcpp::QoS(1));
}

void VehicleReport::can_frame_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    //std_msgs::msg::Header header;
    //header.frame_id = frame_id;

    switch (msg->id)
    {
        case 0x100://Steering
        {
            autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
            steer_msg.stamp = get_clock()->now();
            steer_msg.steering_tire_angle = steer_bytesToFloat(msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
            // RCLCPP_INFO(get_logger(), "Received steering command");
            // RCLCPP_INFO(get_logger(), "Steering mode: %d", msg->data[0]);
            // float steer_deg = steer_bytesToFloat(msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
            // RCLCPP_INFO(get_logger(), "Steering angle: %f", steer_deg);
            steering_report_pub_->publish(steer_msg);
            break;
        }
        case 0x101://throttle
        {
            std_msgs::msg::UInt16 throttle_msg;
            throttle_msg.data = Two_bytesToUint16(msg->data[1], msg->data[2]);
            throttle_report_pub_->publish(throttle_msg);
            // RCLCPP_INFO(get_logger(), "Received steering command");
            // RCLCPP_INFO(get_logger(), "throttle mode: %d", msg->data[0]);
            // RCLCPP_INFO(get_logger(), "throttle msg: %d", throttle_msg.data);
            break;
        }
        case 0x205: //battery
        {
            tier4_vehicle_msgs::msg::BatteryStatus battery_msg;
            battery_msg.stamp = get_clock()->now();
            battery_msg.energy_level = Two_bytesToUint16(msg->data[1], msg->data[2]);
            battery_report_pub_->publish(battery_msg);

            // RCLCPP_INFO(get_logger(), "Received battery command");
            // RCLCPP_INFO(get_logger(), "battery mode: %d", msg->data[0]);
            break;
        }
    }
}

float VehicleReport::steer_bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    uint32_t value = (static_cast<uint32_t>(b3) << 24) |
                     (static_cast<uint32_t>(b2) << 16) |
                     (static_cast<uint32_t>(b1) <<  8) |
                     (static_cast<uint32_t>(b0));
    float result;
    std::memcpy(&result, &value, sizeof(float));
    return result;
}
uint16_t VehicleReport::Two_bytesToUint16(uint8_t b0, uint8_t b1) {
    uint32_t value = (static_cast<uint32_t>(b1) <<  8) |
                     (static_cast<uint32_t>(b0));
    uint16_t result;
    std::memcpy(&result, &value, sizeof(uint16_t));
    return result;
}
} // namespace canedudev_interface
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<canedudev_interface::VehicleReport>());
  rclcpp::shutdown();
  return 0;
}