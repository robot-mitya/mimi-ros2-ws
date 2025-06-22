//
// Created by dmitrydzz on 6/8/25.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "mimi_interfaces/msg/drive_cmd.hpp"
#include "mimi_interfaces/msg/scr_anim_cmd.hpp"
#include "mimi_interfaces/msg/headlights_cmd.hpp"

static constexpr int leftJoystickAxisX = 0;
static constexpr int leftJoystickAxisY = 1;
static constexpr int leftTriggerAxis = 2;
// static constexpr int rightJoystickAxisX = 3;
// static constexpr int rightJoystickAxisY = 4;
static constexpr int rightTriggerAxis = 5;
static constexpr int dPadAxisX = 6;
static constexpr int dPadAxisY = 7;
static constexpr int buttonA = 0;
static constexpr int buttonB = 1;
static constexpr int buttonX = 2;
static constexpr int buttonY = 3;
static constexpr int leftBumperButton = 4;
static constexpr int rightBumperButton = 5;
// static constexpr int backButton = 6;
// static constexpr int startButton = 7;
// static constexpr int guideButton = 8;
// static constexpr int leftStickButton = 9;
// static constexpr int rightStickButton = 10;

struct Vector2 {
    float x;
    float y;
    Vector2() : x(0), y(0) {}
    Vector2(const float _x, const float _y) : x(_x), y(_y) {}
};

struct Color3 {
    float r;
    float g;
    float b;
    Color3() : r(0), g(0), b(0) {}
    Color3(const float r, const float g, const float b) : r(r), g(g), b(b) {}
    bool isSame(const float rr, const float gg, const float bb) const {
        return r == rr && g == gg && b == bb;
    }
    bool isSame(const Color3 other) const {
        return r == other.r && g == other.g && b == other.b;
    }
};

using PublishHeadlightsCmdCallback = std::function<void(Color3 color)>;

class HeadlightsManager {
    PublishHeadlightsCmdCallback publishCallback_ = nullptr;
    uint8_t pressedButtons_ = 0b00000000;
    bool headlightsOn_ = false;
    Color3 srcColor_{1, 1, 1};
    Color3 msgColor_;

    void processTrigger(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        if (!headlightsOn_ && !srcColor_.isSame(0, 0, 0)) {
            const float factor = (1 - msg->axes[leftTriggerAxis]) / 2;
            const Color3 msgColor(srcColor_.r * factor, srcColor_.g * factor, srcColor_.b * factor);
            if (!msgColor.isSame(msgColor_)) {
                msgColor_.r = msgColor.r;
                msgColor_.g = msgColor.g;
                msgColor_.b = msgColor.b;
                publishCallback_(msgColor);
            }
        }
    }

    void processButtons(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        const bool bumperPressed = msg->buttons[leftBumperButton] == 1;
        const bool pressedA = msg->buttons[buttonA] == 1; // green color
        const bool pressedB = msg->buttons[buttonB] == 1; // red color
        const bool pressedX = msg->buttons[buttonX] == 1; // blue color
        const bool pressedY = msg->buttons[buttonY] == 1; // white color
        const uint8_t pressedButtons = pressedA << 0 | pressedB << 1 | pressedX << 2 | pressedY << 3;

        std::optional<Color3> newColor;
        if (pressedButtons != pressedButtons_ && pressedA) {
            newColor.emplace(0, 1, 0);
        } else if (pressedButtons != pressedButtons_ && pressedB) {
            newColor.emplace(1, 0, 0);
        } else if (pressedButtons != pressedButtons_ && pressedX) {
            newColor.emplace(0, 0, 1);
        } else if (pressedButtons != pressedButtons_ && pressedY) {
            newColor.emplace(1, 1, 1);
        }
        pressedButtons_ = pressedButtons;

        if (newColor.has_value()) {
            if (!bumperPressed || headlightsOn_) {
                headlightsOn_ = newColor->isSame(srcColor_) ? !headlightsOn_ : true;

                if (publishCallback_) {
                    const float factor = headlightsOn_ ? 1.0f : 0.0f;
                    const Color3 msgColor((*newColor).r * factor, (*newColor).g * factor, (*newColor).b * factor);
                    if (!msgColor.isSame(msgColor_)) {
                        msgColor_.r = msgColor.r;
                        msgColor_.g = msgColor.g;
                        msgColor_.b = msgColor.b;
                        publishCallback_(msgColor);
                    }
                }
            }
            srcColor_.r = newColor->r;
            srcColor_.g = newColor->g;
            srcColor_.b = newColor->b;
        }
    }
public:
    void setPublishCallback(PublishHeadlightsCmdCallback publishCallback) {
        publishCallback_ = std::move(publishCallback);
    }

    void process(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        processButtons(msg);
        processTrigger(msg);
    }
};

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

    static constexpr int scrAnimEmpty = 0;
    static constexpr int scrAnimBluetooth = 1;
    static constexpr int scrAnimYes = 2;
    static constexpr int scrAnimNo = 3;
    static constexpr int scrAnimHeart = 4;
    static constexpr int scrAnimSpinner = 5;
    static constexpr int scrAnimNormalFace = 6;
    static constexpr int scrAnimHappyFace = 7;
    static constexpr int scrAnimSadFace = 8;
    static constexpr int scrAnimAngryFace = 9;
    static constexpr int scrAnimTalkingFace = 10;

    Color3 headlightsColor_;
    bool headlightsOn_ = false;

    mimi_interfaces::msg::ScrAnimCmd scrAnimMsg_;
    mimi_interfaces::msg::DriveCmd driveMsg_;

    void processControlModeSwitching(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        if (approximatelyEqual(msg->axes[dPadAxisX], -1)) { // D-PAD RIGHT
            if (controlMode_ != ControlMode::Headlights) {
                controlMode_ = ControlMode::Headlights;
                RCLCPP_INFO(this->get_logger(), "Switched to ControlMode::Headlights");
            }
        } else if (approximatelyEqual(msg->axes[dPadAxisY], 1)) { // D-PAD UP
            if (controlMode_ != ControlMode::ScreenAnimations) {
                controlMode_ = ControlMode::ScreenAnimations;
                RCLCPP_INFO(this->get_logger(), "Switched to ControlMode::ScreenAnimations");
            }
        }
    }

    void processScreenAnimationCommands(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        if (controlMode_ != ControlMode::ScreenAnimations) return;
        const bool bumperPressed = msg->buttons[leftBumperButton] == 1;
        const bool pressedA = msg->buttons[buttonA] == 1;
        const bool pressedB = msg->buttons[buttonB] == 1;
        const bool pressedX = msg->buttons[buttonX] == 1;
        const bool pressedY = msg->buttons[buttonY] == 1;
        std::optional<uint32_t> scrAnimId;
        if (bumperPressed) {
            if (pressedA) {
                scrAnimId = scrAnimHeart;
            } else if (pressedB) {
                scrAnimId = scrAnimAngryFace;
            } else if (pressedX) {
                scrAnimId = scrAnimSpinner;
            } else if (pressedY) {
                scrAnimId = scrAnimNo;
            }
        } else {
            if (pressedA) {
                scrAnimId = scrAnimHappyFace;
            } else if (pressedB) {
                scrAnimId = scrAnimSadFace;
            } else if (pressedX) {
                scrAnimId = scrAnimTalkingFace;
            } else if (pressedY) {
                scrAnimId = scrAnimYes;
            }
        }
        if (scrAnimId.has_value() && scrAnimMsg_.id != *scrAnimId) {
            scrAnimMsg_.id = *scrAnimId;
            publisherScrAnimCmd_->publish(scrAnimMsg_);
        }
    }

    HeadlightsManager headlightsManager_;

    void processDriveCommands(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        const Vector2 joystickVector(-msg->axes[leftJoystickAxisX], msg->axes[leftJoystickAxisY]);
        const Vector2 moveVector = CircleToSquare(joystickVector);
        const bool fastRotation = msg->buttons[rightBumperButton] > 0;
        const double speedFactor = fastRotation ? 1.0 : (1 - msg->axes[rightTriggerAxis]) / 2;
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
        this->headlightsManager_.setPublishCallback([this] (const Color3 color) {
            mimi_interfaces::msg::HeadlightsCmd message;
            message.red = color.r;
            message.green = color.g;
            message.blue = color.b;
            publisherHeadlightsCmd_->publish(message);
        });

        auto joyTopicCallback = [this](const sensor_msgs::msg::Joy::UniquePtr &msg) -> void {
            this->processControlModeSwitching(msg);
            this->processScreenAnimationCommands(msg);
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
