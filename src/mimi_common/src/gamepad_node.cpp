//
// Created by dmitrydzz on 6/8/25.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "types.h"
#include "screen_animation_manager.h"
#include "headlights_manager.h"
#include "mimi_interfaces/msg/drive_cmd.hpp"
#include "mimi_interfaces/msg/headlights_cmd.hpp"

class GamepadNode final : public rclcpp::Node {
    static constexpr double maxSpeed = 1.0;

    enum class ControlMode {
        ScreenAnimations,
        Headlights,
    };

    ControlMode controlMode_ = ControlMode::ScreenAnimations;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriptionJoy_;
    rclcpp::Publisher<mimi_interfaces::msg::DriveCmd>::SharedPtr publisherDriveCmd_;
    rclcpp::Publisher<mimi_interfaces::msg::ScrAnimCmd>::SharedPtr publisherScrAnimCmd_;
    rclcpp::Publisher<mimi_interfaces::msg::HeadlightsCmd>::SharedPtr publisherHeadlightsCmd_;

    static Vector2 CircleToSquareInFirstQuadrant(Vector2 value) {
        float x = value.x;
        float y = value.y;
        if (x == 0) return value; // (to avoid dividing by 0)
        if (x >= 0 && y >= 0) {
            const bool firstOctantInQuadrant = x >= y;
            if (!firstOctantInQuadrant) std::swap(x, y);
            const float resultX = sqrtf(x * x + y * y);
            const float resultY = y * resultX / x;
            value.x = resultX;
            value.y = resultY;
            if (!firstOctantInQuadrant) std::swap(value.x, value.y);
        }
        return value;
    }

    static Vector2 CircleToSquare(Vector2 value) {
        if (value.x >= 0 && value.y >= 0) {
            value = CircleToSquareInFirstQuadrant(value);
        } else if (value.x < 0 && value.y >= 0) {
            value.x = -value.x;
            value = CircleToSquareInFirstQuadrant(value);
            value.x = -value.x;
        } else if (value.x < 0 && value.y < 0) {
            value.x = -value.x;
            value.y = -value.y;
            value = CircleToSquareInFirstQuadrant(value);
            value.x = -value.x;
            value.y = -value.y;
        } else if (value.x >= 0 && value.y < 0) {
            value.y = -value.y;
            value = CircleToSquareInFirstQuadrant(value);
            value.y = -value.y;
        }
        value.x = std::clamp(value.x, -1.0f, 1.0f);
        value.y = std::clamp(value.y, -1.0f, 1.0f);
        return value;
    }

    static Vector2 GetMotorValues(const Vector2 moveVector, const float speedFactor, const bool fastRotation) {
        const double leftSpeedFactor = fastRotation
                                     ? moveVector.x / 2.0
                                     : moveVector.x < 0.0
                                           ? 1.0 + moveVector.x
                                           : 1.0;
        const double rightSpeedFactor = fastRotation
                                      ? -moveVector.x / 2.0
                                      : moveVector.x > 0.0
                                            ? 1.0 - moveVector.x
                                            : 1.0;
        const double directionSpeedFactor = fastRotation ? 1.0 : moveVector.y;

        const double leftSpeed = speedFactor * directionSpeedFactor * leftSpeedFactor * maxSpeed;
        const double rightSpeed = speedFactor * directionSpeedFactor * rightSpeedFactor * maxSpeed;
        return {static_cast<float>(leftSpeed), static_cast<float>(rightSpeed)};
    }

    static bool approximatelyEqual(const float a, const float b) {
        return std::fabs(a - b) < 1e-3f;
    }

    Color3 headlightsColor_;
    bool headlightsOn_ = false;

    mimi_interfaces::msg::DriveCmd driveMsg_;

    void processControlModeSwitching(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        if (approximatelyEqual(msg->axes[Xbox::dPadAxisX], -1)) { // D-PAD RIGHT
            if (controlMode_ != ControlMode::Headlights) {
                controlMode_ = ControlMode::Headlights;
                RCLCPP_INFO(this->get_logger(), "Switched to ControlMode::Headlights");
            }
        } else if (approximatelyEqual(msg->axes[Xbox::dPadAxisY], 1)) { // D-PAD UP
            if (controlMode_ != ControlMode::ScreenAnimations) {
                controlMode_ = ControlMode::ScreenAnimations;
                RCLCPP_INFO(this->get_logger(), "Switched to ControlMode::ScreenAnimations");
            }
        }
    }

    ScreenAnimationManager screenAnimationManager_;
    HeadlightsManager headlightsManager_;

    void processDriveCommands(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        const Vector2 joystickVector(-msg->axes[Xbox::leftJoystickAxisX], msg->axes[Xbox::leftJoystickAxisY]);
        const Vector2 moveVector = CircleToSquare(joystickVector);
        const bool fastRotation = msg->buttons[Xbox::rightBumperButton] > 0;
        const double speedFactor = fastRotation ? 1.0 : (1 - msg->axes[Xbox::rightTriggerAxis]) / 2;
        const Vector2 motorValues = GetMotorValues(moveVector, static_cast<float>(speedFactor), fastRotation);

        // driveMsg_.left_speed = motorValues.x;
        // driveMsg_.right_speed = motorValues.y;
        // publisherDriveCmd_->publish(driveMsg_);
        if (!approximatelyEqual(driveMsg_.left_speed, motorValues.x) || !approximatelyEqual(driveMsg_.right_speed, motorValues.y)) {
            driveMsg_.left_speed = motorValues.x;
            driveMsg_.right_speed = motorValues.y;
            publisherDriveCmd_->publish(driveMsg_);
        }
    }
public:
    GamepadNode() : Node("gamepad_node") {
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
            if (controlMode_ == ControlMode::ScreenAnimations) {
                this->screenAnimationManager_.process(msg);
            }
            if (controlMode_ == ControlMode::Headlights) {
                this->headlightsManager_.process(msg);
            }
            this->processDriveCommands(msg);
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
