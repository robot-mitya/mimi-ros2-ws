#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "mimi_ble/ble_uart_client.h"
#include "mimi_interfaces/msg/drive_cmd.hpp"
#include "mimi_interfaces/msg/scr_anim_cmd.hpp"
#include "mimi_interfaces/msg/headlights_cmd.hpp"

std::shared_ptr<mimi::BleUartClient> bleClient;

class BleNode final : public rclcpp::Node {
    static constexpr double maxSpeed = 1.0;

    std::string _robotName;

    rclcpp::Subscription<mimi_interfaces::msg::DriveCmd>::SharedPtr driveCmdSubscription_;
    rclcpp::Subscription<mimi_interfaces::msg::ScrAnimCmd>::SharedPtr scrAnimCmdSubscription_;
    rclcpp::Subscription<mimi_interfaces::msg::HeadlightsCmd>::SharedPtr headlightsCmdSubscription_;

    static int16_t toInt255(double value) {
        value = std::clamp(value, -1.0, 1.0);
        return static_cast<int16_t>(std::round(value * 255.0));
    }

    static uint8_t toUint255(double value) {
        value = std::clamp(value, 0.0, 1.0);
        return static_cast<uint8_t>(std::round(value * 255.0));
    }

public:
    BleNode() : Node("ble_node") {
        _robotName = declare_parameter("robot_name", "mimi");
        get_parameter("robot_name", _robotName);

        auto driveCmdTopicCallback = [this](const mimi_interfaces::msg::DriveCmd::UniquePtr &msg) -> void {
            if (bleClient->getState() != mimi::BleUartClient::State::Connected) return;
            const int16_t leftSpeed = toInt255(msg->left_speed);
            const int16_t rightSpeed = toInt255(msg->right_speed);
            const std::string command = mimi::str("drv ", leftSpeed, " ", rightSpeed, "\r\n");
            if (!bleClient->send(command)) {
                RCLCPP_ERROR(this->get_logger(), "Sending command failed: %s", command.c_str());
            }
        };

        auto scrAnimCmdTopicCallback = [this](const mimi_interfaces::msg::ScrAnimCmd::UniquePtr &msg) -> void {
            if (bleClient->getState() != mimi::BleUartClient::State::Connected) return;
            const std::string command = mimi::str("emo ", static_cast<int>(msg->id), "\r\n");
            if (!bleClient->send(command)) {
                RCLCPP_ERROR(this->get_logger(), "Sending command failed: %s", command.c_str());
            }
        };

        auto headlightsCmdTopicCallback = [this](const mimi_interfaces::msg::HeadlightsCmd::UniquePtr &msg) -> void {
            if (bleClient->getState() != mimi::BleUartClient::State::Connected) return;
            const int red = toUint255(msg->red);
            const int green = toUint255(msg->green);
            const int blue = toUint255(msg->blue);
            const std::string command = mimi::str("hl ", red, " ", green, " ", blue, "\r\n");
            if (!bleClient->send(command)) {
                RCLCPP_ERROR(this->get_logger(), "Sending command failed: %s", command.c_str());
            }
        };

        driveCmdSubscription_ = this->create_subscription<mimi_interfaces::msg::DriveCmd>("/cmd/drive_cmd", 10, driveCmdTopicCallback);
        scrAnimCmdSubscription_ = this->create_subscription<mimi_interfaces::msg::ScrAnimCmd>("/cmd/scr_anim_cmd", 10, scrAnimCmdTopicCallback);
        headlightsCmdSubscription_ = this->create_subscription<mimi_interfaces::msg::HeadlightsCmd>("/cmd/headlights_cmd", 10, headlightsCmdTopicCallback);
    }

    std::string& getRobotName() {
        return _robotName;
    }
};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<BleNode>();
    // rclcpp::Parameter paramRobotName = node->get_parameter("robot_name");

    bleClient = std::make_shared<mimi::BleUartClient>();
    bleClient->setCallbacks(
        [node](const std::string& /*deviceAlias*/, const std::string& connectedText, const bool /*afterFailure*/) {
            RCLCPP_INFO(node->get_logger(), "OnConnect: %s", connectedText.c_str());
        },
        [node](const std::string& /*deviceAlias*/, const std::string& disconnectedText, const bool /*isFailure*/) {
            RCLCPP_INFO(node->get_logger(), "OnDisconnect: %s", disconnectedText.c_str());
        },
        [node](const std::string& /*deviceAlias*/, const mimi::BleUartClient::State& state) {
            RCLCPP_INFO(node->get_logger(), "OnStateChanged: %s", mimi::BleUartClient::stateToString(state));
        },
        [node](const std::string& /*deviceAlias*/, const std::string& errorText, const std::string& /*sdbusErrorName*/, const mimi::BleUartClient::State& /*state*/) {
            RCLCPP_ERROR(node->get_logger(), "OnError: %s", errorText.c_str());
        },
        [node](const std::string& /*deviceAlias*/, const std::string& receivedMessage) {
            RCLCPP_INFO(node->get_logger(), "OnReceived: %s", receivedMessage.c_str());
        }
    );
    // bleClient->connect("BBC micro:bit", true);
    bleClient->connect(node->getRobotName(), true);

    rclcpp::Rate rate(20);  // 50ms
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        bleClient->processCallbacks();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
