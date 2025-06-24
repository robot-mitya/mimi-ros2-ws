//
// Created by dmitrydzz on 6/8/25.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "types.h"
#include "control_mode_manager.h"
#include "drive_manager.h"
#include "screen_animation_manager.h"
#include "headlights_manager.h"
#include "mimi_interfaces/msg/drive_cmd.hpp"
#include "mimi_interfaces/msg/headlights_cmd.hpp"

class GamepadNode final : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriptionJoy_;
    rclcpp::Publisher<mimi_interfaces::msg::DriveCmd>::SharedPtr publisherDriveCmd_;
    rclcpp::Publisher<mimi_interfaces::msg::ScrAnimCmd>::SharedPtr publisherScrAnimCmd_;
    rclcpp::Publisher<mimi_interfaces::msg::HeadlightsCmd>::SharedPtr publisherHeadlightsCmd_;

    ControlModeManager controlModeManager_;
    DriveManager driveManager_;
    ScreenAnimationManager screenAnimationManager_;
    HeadlightsManager headlightsManager_;
public:
    GamepadNode() : Node("gamepad_node") {
        this->controlModeManager_.setControlModeChangedCallback([this] (const ControlMode& controlMode) {
            std::optional<std::string> modeName;
            switch (controlMode) {
                case ControlMode::ScreenAnimations: modeName = "ScreenAnimations"; break;
                case ControlMode::Headlights: modeName = "Headlights"; break;
            }
            if (modeName) {
                RCLCPP_INFO(this->get_logger(), "Switched to %s mode", modeName->c_str());
            }
        });

        this->driveManager_.setPublishCallback([this] (const mimi_interfaces::msg::DriveCmd driveMsg) {
            publisherDriveCmd_->publish(driveMsg);
        });

        this->screenAnimationManager_.setPublishCallback([this] (const mimi_interfaces::msg::ScrAnimCmd scrAnimMsg) {
            publisherScrAnimCmd_->publish(scrAnimMsg);
        });

        this->headlightsManager_.setPublishCallback([this] (const Color3 color) {
            mimi_interfaces::msg::HeadlightsCmd message;
            message.red = color.r;
            message.green = color.g;
            message.blue = color.b;
            publisherHeadlightsCmd_->publish(message);
        });

        auto joyTopicCallback = [this](const sensor_msgs::msg::Joy::UniquePtr &msg) -> void {
            this->controlModeManager_.process(msg);
            const ControlMode controlMode = this->controlModeManager_.getControlMode();

            this->driveManager_.process(msg);

            if (controlMode == ControlMode::ScreenAnimations) {
                this->screenAnimationManager_.process(msg);
            }

            if (controlMode == ControlMode::Headlights) {
                this->headlightsManager_.process(msg);
            }
        };

        publisherDriveCmd_ = this->create_publisher<mimi_interfaces::msg::DriveCmd>("/cmd/drive_cmd", 10);
        publisherScrAnimCmd_ = this->create_publisher<mimi_interfaces::msg::ScrAnimCmd>("/cmd/scr_anim_cmd", 10);
        publisherHeadlightsCmd_ = this->create_publisher<mimi_interfaces::msg::HeadlightsCmd>("/cmd/headlights_cmd", 10);
        subscriptionJoy_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, joyTopicCallback);
    }
};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamepadNode>());
    rclcpp::shutdown();
    return 0;
}
