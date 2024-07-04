#include "rclcpp/rclcpp.hpp"
#include <can_msgs/msg/frame.hpp>
#include <stdint.h>
#include <memory>
#include <string>
#include <std_msgs/msg/u_int16.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <cmath>
namespace canedudev_interface
{
class VehicleReport : public rclcpp::Node
{
    public:
        //Constructor
        VehicleReport();
        uint16_t Two_bytesToUint16(uint8_t b0, uint8_t b1);
        float steer_bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
    private:
        double loop_rate_;
        std::string frame_id_;
        float steer_angle_ = 0.0;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_report_pub_;
        rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_report_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_mode_report_pub_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_report_pub_;
        void can_frame_callback(const can_msgs::msg::Frame::SharedPtr msg);


};  
} // namespace canedudev_interface