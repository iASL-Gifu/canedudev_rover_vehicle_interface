#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class SenderBridge : public rclcpp::Node {
public:
    SenderBridge()
    : Node("sender_bridge"), override_active_(false), joy_steer_(0.0), joy_throttle_(50.0), latest_throttle_(50.0), latest_steering_(0.0)
    {
        // パラメータの宣言と取得
        this->declare_parameter("timer_period_ms", 100);

        // ジョイメッセージのサブスクリプションの設定
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 1, std::bind(&SenderBridge::joy_callback, this, std::placeholders::_1));

        // スロットルとステアリングのサブスクリプションの設定
        throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "throttle_sub", 1, std::bind(&SenderBridge::throttle_callback, this, std::placeholders::_1));
        steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "steering_sub", 1, std::bind(&SenderBridge::steering_callback, this, std::placeholders::_1));

        // パブリッシャーの設定
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("throttle_pub", 1);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("steering_pub", 1);
        button_10_pub_ = this->create_publisher<std_msgs::msg::Bool>("battery_pub", 1);

        // 初期状態の値をパブリッシュ
        publish_initial_values();
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ボタンのインデックスを指定してチェック（例：ボタン4）
        if (msg->buttons[4] == 1)
        {
            override_active_ = true;
            joy_steer_ = scale_steering(msg->axes[0]); // 左スティックのx軸をステアリングに使用
            joy_throttle_ = scale_throttle(msg->axes[3]); // 右スティックのy軸をスロットルに使用

            // デバッグメッセージの追加
            RCLCPP_INFO(this->get_logger(), "Joy input - Steering: %f, Throttle: %f", msg->axes[0], msg->axes[4]);
            RCLCPP_INFO(this->get_logger(), "Scaled values - Steering: %f, Throttle: %f", joy_steer_, joy_throttle_);
            
            publish_control_values(joy_throttle_, joy_steer_);
        }
        else
        {
            override_active_ = false;
        }

        // button[10]のチェックとBoolメッセージの送信
        if (msg->buttons.size() > 10 && msg->buttons[8] == 1 && msg->buttons[9] == 1)
        {
            auto bool_msg = std_msgs::msg::Bool();
            bool_msg.data = true;
            button_10_pub_->publish(bool_msg);
            RCLCPP_INFO(this->get_logger(), "Button 9 and 10 are pressed, sending True.");
        }
    }

    void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!override_active_) {
            latest_throttle_ = msg->data;
            publish_control_values(latest_throttle_, latest_steering_);
        }
    }

    void steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!override_active_) {
            latest_steering_ = msg->data;
            publish_control_values(latest_throttle_, latest_steering_);
        }
    }

    void publish_initial_values()
    {
        publish_control_values(50.0, 0.0); // 初期状態をニュートラルに設定
    }

    void publish_control_values(float throttle, float steering)
    {
        std_msgs::msg::Float32 throttle_msg;
        throttle_msg.data = throttle;
        throttle_pub_->publish(throttle_msg);

        std_msgs::msg::Float32 steering_msg;
        steering_msg.data = steering;
        steering_pub_->publish(steering_msg);
    }

    // スロットルのスケーリング
    float scale_throttle(float value)
    {
        if (value < 0.0f) { // 後退の場合
            return 50.0f + (value * 30.0f); // -1.0から0.0を20.0から50.0にマッピング
        } else { // 前進の場合
            return 50.0f + (value * 50.0f); // 0.0から1.0を50.0から100.0にマッピング
        }
    }

    // ステアリングのスケーリング (-1.0〜1.0 を -45〜45 度に変換)
    float scale_steering(float value)
    {
        return value * 45.0f;
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_10_pub_;

    bool override_active_;
    float joy_steer_;
    float joy_throttle_;
    float latest_throttle_;
    float latest_steering_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SenderBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
