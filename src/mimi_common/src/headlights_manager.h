#ifndef HEADLIGHTS_MANAGER_H
#define HEADLIGHTS_MANAGER_H

#include "types.h"

using PublishHeadlightsCmdCallback = std::function<void(Color3& color)>;

class HeadlightsManager {
    PublishHeadlightsCmdCallback publishCallback_ = nullptr;
    uint8_t pressedButtons_ = 0b00000000;
    bool headlightsOn_ = false;
    Color3 srcColor_{1, 1, 1};
    Color3 msgColor_;

    std::optional<rclcpp::Time> lastProcessTriggerTime_;
    rclcpp::Duration minProcessTriggerPublishInterval_{0, 50000000}; // 50 ms = 20 Hz

    void processTrigger(rclcpp::Node* node, const sensor_msgs::msg::Joy::UniquePtr &msg) {
        if (publishCallback_ && !headlightsOn_ && !srcColor_.isSame(0, 0, 0)) {
            const auto now = node->get_clock()->now();
            if (!lastProcessTriggerTime_.has_value() || now - *lastProcessTriggerTime_ > minProcessTriggerPublishInterval_) {
                const float factor = (1 - msg->axes[Xbox::leftTriggerAxis]) / 2;
                const Color3 msgColor(srcColor_.r * factor, srcColor_.g * factor, srcColor_.b * factor);
                if (!msgColor.isSame(msgColor_)) {
                    msgColor_.r = msgColor.r;
                    msgColor_.g = msgColor.g;
                    msgColor_.b = msgColor.b;
                    publishCallback_(msgColor_);
                }

                lastProcessTriggerTime_ = now;
            }
        }
    }

    void processButtons(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        const bool bumperPressed = msg->buttons[Xbox::leftBumperButton] == 1;
        const bool pressedA = msg->buttons[Xbox::buttonA] == 1; // green color
        const bool pressedB = msg->buttons[Xbox::buttonB] == 1; // red color
        const bool pressedX = msg->buttons[Xbox::buttonX] == 1; // blue color
        const bool pressedY = msg->buttons[Xbox::buttonY] == 1; // white color
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
                    const Color3 msgColor(newColor->r * factor, newColor->g * factor, newColor->b * factor);
                    if (!msgColor.isSame(msgColor_)) {
                        msgColor_.r = msgColor.r;
                        msgColor_.g = msgColor.g;
                        msgColor_.b = msgColor.b;
                        publishCallback_(msgColor_);
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

    void process(rclcpp::Node* node, const sensor_msgs::msg::Joy::UniquePtr &msg) {
        processButtons(msg);
        processTrigger(node, msg);
    }
};

#endif //HEADLIGHTS_MANAGER_H
