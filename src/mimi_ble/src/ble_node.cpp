#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mimi_interfaces/msg/drive.hpp"

class BleNode final : public rclcpp::Node {
    static constexpr double maxSpeed = 1.0;

    rclcpp::Subscription<mimi_interfaces::msg::Drive>::SharedPtr driveSubscription_;

    static int16_t toInt255(double value) {
        value = std::clamp(value, -1.0, 1.0);            // ограничиваем в пределах [-1.0, 1.0]
        return static_cast<int16_t>(std::round(value * 255.0));
    }

public:
    BleNode()
        : Node("ble_node") {
        auto driveTopicCallback = [this](const mimi_interfaces::msg::Drive::UniquePtr &msg) -> void {
            const int16_t leftSpeed = toInt255(msg->left_speed);
            const int16_t rightSpeed = toInt255(msg->right_speed);
            RCLCPP_INFO(this->get_logger(), "Motor values: [%4d, %4d]", leftSpeed, rightSpeed);
        };
        driveSubscription_ = this->create_subscription<mimi_interfaces::msg::Drive>("/drive", 10, driveTopicCallback);
    }
};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BleNode>());
    rclcpp::shutdown();
    return 0;
}
