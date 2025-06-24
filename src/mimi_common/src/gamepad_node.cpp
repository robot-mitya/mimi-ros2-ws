//
// Created by dmitrydzz on 6/8/25.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "types.h"
#include "drive_manager.h"
#include "screen_animation_manager.h"
#include "headlights_manager.h"
#include "mimi_interfaces/msg/drive_cmd.hpp"
#include "mimi_interfaces/msg/headlights_cmd.hpp"

class GamepadNode final : public rclcpp::Node {
    enum class ControlMode {
        ScreenAnimations,
        Headlights,
    };

    ControlMode controlMode_ = ControlMode::ScreenAnimations;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriptionJoy_;
    rclcpp::Publisher<mimi_interfaces::msg::DriveCmd>::SharedPtr publisherDriveCmd_;
    rclcpp::Publisher<mimi_interfaces::msg::ScrAnimCmd>::SharedPtr publisherScrAnimCmd_;
    rclcpp::Publisher<mimi_interfaces::msg::HeadlightsCmd>::SharedPtr publisherHeadlightsCmd_;

    Color3 headlightsColor_;
    bool headlightsOn_ = false;

    void processControlModeSwitching(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        if (CommonHelper::approximatelyEqual(msg->axes[Xbox::dPadAxisX], -1)) { // D-PAD RIGHT
            if (controlMode_ != ControlMode::Headlights) {
                controlMode_ = ControlMode::Headlights;
                RCLCPP_INFO(this->get_logger(), "Switched to ControlMode::Headlights");
            }
        } else if (CommonHelper::approximatelyEqual(msg->axes[Xbox::dPadAxisY], 1)) { // D-PAD UP
            if (controlMode_ != ControlMode::ScreenAnimations) {
                controlMode_ = ControlMode::ScreenAnimations;
                RCLCPP_INFO(this->get_logger(), "Switched to ControlMode::ScreenAnimations");
            }
        }
    }

    DriveManager driveManager_;
    ScreenAnimationManager screenAnimationManager_;
    HeadlightsManager headlightsManager_;
public:
    GamepadNode() : Node("gamepad_node") {
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
            this->processControlModeSwitching(msg);

            this->driveManager_.process(msg);

            if (controlMode_ == ControlMode::ScreenAnimations) {
                this->screenAnimationManager_.process(msg);
            }

            if (controlMode_ == ControlMode::Headlights) {
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
