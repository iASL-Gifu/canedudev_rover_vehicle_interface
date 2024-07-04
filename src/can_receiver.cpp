#include <canedudev_interface/can_receiver.hpp>
namespace canedudev_interface
{
VehicleReport::VehicleReport(): Node("canedudev_vehicle_report_node")
{
    frame_id_  = declare_parameter("frame_id", "base_link");
    loop_rate_ = declare_parameter("loop_rate", 50.0);
    //Subscription
    can_frame_sub_ = create_subscription<can_msgs::msg::Frame>(
        "/input/can_rx", 10, std::bind(&canedudev_interface::VehicleReport::can_frame_callback, this, std::placeholders::_1));
    steering_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS(1));
    velocity_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS(1));
    battery_report_pub_  = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", rclcpp::QoS(1));
    gear_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS(1));
    control_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS(1));
}

void VehicleReport::can_frame_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    //std_msgs::msg::Header header;
    //header.frame_id = frame_id;

    switch (msg->id)
    {
        case 0x100: //Steering
        {
            steer_angle_ = steer_bytesToFloat(msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
            autoware_vehicle_msgs::msg::SteeringReport steer_msg;
            steer_msg.stamp = get_clock()->now();
            steer_msg.steering_tire_angle =  steer_angle_ * 3.14 / 180 / 17;
            // RCLCPP_INFO(get_logger(), "Received steering command");
            // RCLCPP_INFO(get_logger(), "Steering mode: %d", msg->data[0]);
            // float steer_deg = steer_bytesToFloat(msg->data[1], msg->data[2], msg->data[3], msg->data[4]);
            // RCLCPP_INFO(get_logger(), "Steering angle: %f", steer_deg);
            steering_report_pub_->publish(steer_msg);
            break;
        }
        case 0x101: //throttle
        {
            autoware_vehicle_msgs::msg::VelocityReport vel_report_msg;
            autoware_vehicle_msgs::msg::GearReport gear_report_msg;
            vel_report_msg.header.stamp = get_clock()->now();
            gear_report_msg.stamp       = get_clock()->now();

            uint16_t vel_rpm;
            vel_rpm = Two_bytesToUint16(msg->data[1], msg->data[2]);
            // RCLCPP_INFO(get_logger(), "Velocity rpm: %d", vel_rpm);

            // TO-DO Separate Drive and Reverse
            float velocity;
            if (vel_rpm == 1500){
                velocity = 0;
                gear_report_msg.report = 22; //park
            }
            else{
                velocity = 30 / (500 * abs(vel_rpm -1500) * 3.6);
                gear_report_msg.report = 2; //drive or reverse()
            // RCLCPP_INFO(get_logger(), "Velocity: %f", velocity);  
            }
            //vel_report_msg.longitudinal_velocity = velocity * cos(steer_angle_);
            //vel_report_msg.lateral_velocity      = velocity * sin(steer_angle_);
            RCLCPP_INFO(get_logger(), "Velocity: %f", velocity);
            vel_report_msg.longitudinal_velocity = velocity;
            vel_report_msg.lateral_velocity      = 0;
            vel_report_msg.heading_rate          = 0;
            vel_report_msg.header.frame_id       = "base_link";

            gear_mode_report_pub_->publish(gear_report_msg);
            velocity_report_pub_ ->publish(vel_report_msg);
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
        case 0x203: //Servo Voltage
            break;
        case 0x204: //Servo Current
            break;
        case 0x200: //BATTERY_CELL_VOLTAGE
            break;
        case 0x201: //BATTERY_REGURATED_OUTPUT_ENVELOPE
            break;
        case 0x202: //BATTERY_BATTERT_OUTPUT_ENVELOPE
            break;
        default:
            RCLCPP_INFO(get_logger(), "Unknown CAN ID: %d", msg->id);
            break;
    }
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = get_clock()->now();
    control_mode_report.mode = 1;// Always autonomous
    control_mode_report_pub_->publish(control_mode_report);
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