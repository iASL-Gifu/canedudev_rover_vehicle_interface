#include "rclcpp/rclcpp.hpp"
#include <can_msgs/msg/frame.hpp>
#include <stdint.h>
#include <memory>
#include <string>
#include <std_msgs/msg/u_int16.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
namespace canedudev_interface
{
class VehicleReport : public rclcpp::Node
{
    public:
        //Constructor
        VehicleReport();
        uint16_t throttle_bytesToFloat(uint8_t b0, uint8_t b1);
        float steer_bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);
    private:
        double loop_rate_;
        std::string frame_id_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_frame_sub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_report_pub_;
        rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr throttle_report_pub_;
        void can_frame_callback(const can_msgs::msg::Frame::SharedPtr msg);


};  
} // namespace canedudev_interface