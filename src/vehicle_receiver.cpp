#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"
#include "autoware_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"  // Include the correct message type for the velocity

namespace canedudev_interface
{
class VehicleReport : public rclcpp::Node {
public:
    VehicleReport();
    ~VehicleReport() = default;

private:
    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);  // Update the message type
    void timer_callback();

    std::string frame_id_;
    double loop_rate_;
    float steer_angle_;
    float longitudinal_velocity_;
    float lateral_velocity_;
    float velocity_threshold_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;  // Update the subscription type
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;
    rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_mode_report_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

VehicleReport::VehicleReport() : Node("canedudev_vehicle_report_node"), longitudinal_velocity_(0.0), lateral_velocity_(0.0)  // Initialize velocities to 0
{
    frame_id_  = declare_parameter("frame_id", "base_link");
    loop_rate_ = declare_parameter("loop_rate", 300.0);
    velocity_threshold_ = declare_parameter("velocity_threshold", 0.1);  // Declare the velocity threshold parameter

    // Subscription
    steering_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/input/steering", 1, std::bind(&VehicleReport::steering_callback, this, std::placeholders::_1));
    velocity_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/input/velocity", 1, std::bind(&VehicleReport::velocity_callback, this, std::placeholders::_1));  // Update the subscription

    steering_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", rclcpp::QoS(1));
    velocity_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", rclcpp::QoS(1));
    battery_report_pub_  = create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>("/vehicle/status/battery_charge", rclcpp::QoS(1));
    gear_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", rclcpp::QoS(1));
    control_mode_report_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", rclcpp::QoS(1));

    timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / loop_rate_)), std::bind(&VehicleReport::timer_callback, this));
}

void VehicleReport::steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    steer_angle_ = msg->data;
}

void VehicleReport::velocity_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)  // Update the callback
{
    longitudinal_velocity_ = (std::abs(msg->vector.x) > velocity_threshold_) ? msg->vector.x : 0.0;  // Apply the velocity threshold
    lateral_velocity_ = (std::abs(msg->vector.y) > velocity_threshold_) ? msg->vector.y : 0.0;  // Apply the velocity threshold
}

void VehicleReport::timer_callback()
{
    // Publish steering report
    autoware_vehicle_msgs::msg::SteeringReport steer_msg;
    steer_msg.stamp = get_clock()->now();
    steer_msg.steering_tire_angle = steer_angle_ * M_PI / 180.0;  // Convert to radians
    steering_report_pub_->publish(steer_msg);

    // Publish velocity report
    autoware_vehicle_msgs::msg::VelocityReport vel_report_msg;
    vel_report_msg.header.stamp = get_clock()->now();
    vel_report_msg.longitudinal_velocity = longitudinal_velocity_;  // Use the received longitudinal velocity
    vel_report_msg.lateral_velocity = lateral_velocity_;  // Use the received lateral velocity
    vel_report_msg.heading_rate = 0;
    vel_report_msg.header.frame_id = frame_id_;
    velocity_report_pub_->publish(vel_report_msg);

    // Publish control mode report
    autoware_vehicle_msgs::msg::ControlModeReport control_mode_report;
    control_mode_report.stamp = get_clock()->now();
    control_mode_report.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
    control_mode_report_pub_->publish(control_mode_report);
}

}  // namespace canedudev_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<canedudev_interface::VehicleReport>());
  rclcpp::shutdown();
  return 0;
}
