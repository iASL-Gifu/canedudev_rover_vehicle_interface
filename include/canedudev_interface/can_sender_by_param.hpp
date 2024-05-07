#include "rclcpp/rclcpp.hpp"
#include <can_msgs/msg/frame.hpp>
#include <stdint.h>
#include <memory>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
namespace canedudev_interface
{
using rcl_interfaces::msg::SetParametersResult;
class ControlCommandbyParam : public rclcpp::Node
{
    public:
        //Constructor
        ControlCommandbyParam();

    private:
        can_msgs::msg::Frame::ConstSharedPtr steer_ctrl_can_ptr_;
        can_msgs::msg::Frame::ConstSharedPtr throttle_ctrl_can_ptr_;
        rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_frame_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        uint16_t throttle_cmd_;
        float steer_cmd_;
        double loop_rate_;
        OnSetParametersCallbackHandle::SharedPtr set_param_res_; 
        float steer_bytesToFloat(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);

        /**
         * @brief Actuation command callback (Include throttle, brake, steering)
         * 
         * @param msg tier4_vehicle_msgs::msg::ActuationCommandStamped
         */
        void actuation_callback();
        /**
         * @brief Timer callback for publish CAN message 
         * 
         */
        void timer_callback();
        SetParametersResult onParameter(const std::vector<rclcpp::Parameter> & parameters);
        float declare_parameter_with_min_max(
          const std::string & name, const float default_value, const float min_value,
          const float max_value, const std::string & description_name, const std::string & description);
};
} // namespace canedudev_interface