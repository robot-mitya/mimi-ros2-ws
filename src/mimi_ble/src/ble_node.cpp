#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mimi_interfaces/msg/drive.hpp"
#include "mimi_ble/ble_uart_client.h"

std::shared_ptr<mimi::BleUartClient> bleClient;

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
            std::string driveCommand = mimi::str("drv ", leftSpeed, " ", rightSpeed, "\r\n");
            // RCLCPP_INFO(this->get_logger(), "Sending command: %s", driveCommand.c_str());
            if (!bleClient->send(driveCommand.c_str())) {
                RCLCPP_ERROR(this->get_logger(), "Sending command failed");
            }
        };
        driveSubscription_ = this->create_subscription<mimi_interfaces::msg::Drive>("/drive", 10, driveTopicCallback);
    }
};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<BleNode>();

    bleClient = std::make_shared<mimi::BleUartClient>(
        [node](const std::string& deviceAlias, const std::string& connectedText, const bool afterFailure) {
            RCLCPP_INFO(node->get_logger(), "%s", connectedText.c_str());
        },
        [node](const std::string& deviceAlias, const std::string& disconnectedText, const bool isFailure) {
            RCLCPP_INFO(node->get_logger(), "%s", disconnectedText.c_str());
        },
        [node](const std::string& deviceAlias, const std::string& errorText, const std::string& sdbusErrorName, const bool isConnected) {
            RCLCPP_ERROR(node->get_logger(), "%s", errorText.c_str());
        },
        [node](const std::string& deviceAlias, const std::string& receivedMessage) {
            RCLCPP_INFO(node->get_logger(), "Received: %s", receivedMessage.c_str());
        }
    );
    bleClient->connect("BBC micro:bit", true);

    rclcpp::Rate rate(20);  // 50ms
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        bleClient->processCallbacks();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
