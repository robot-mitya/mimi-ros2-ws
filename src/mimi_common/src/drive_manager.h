#ifndef DRIVE_MANAGER_H
#define DRIVE_MANAGER_H

#include "types.h"
#include "mimi_interfaces/msg/drive_cmd.hpp"

using PublishDriveCmdCallback = std::function<void(mimi_interfaces::msg::DriveCmd& driveMsg)>;

class DriveManager {
    PublishDriveCmdCallback publishCallback_ = nullptr;

    std::optional<rclcpp::Time> lastProcessTime_;
    rclcpp::Duration minProcessPublishInterval_{0, 50000000}; // 50 ms = 20 Hz

    mimi_interfaces::msg::DriveCmd driveMsg_;

    static constexpr double maxSpeed = 1.0;

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
public:
    void setPublishCallback(PublishDriveCmdCallback publishCallback) {
        publishCallback_ = std::move(publishCallback);
    }

    void process(rclcpp::Node* node, const sensor_msgs::msg::Joy::UniquePtr &msg) {
        const Vector2 joystickVector(-msg->axes[Xbox::leftJoystickAxisX], msg->axes[Xbox::leftJoystickAxisY]);
        const Vector2 moveVector = CommonHelper::CircleToSquare(joystickVector);
        const bool fastRotation = msg->buttons[Xbox::rightBumperButton] > 0;
        const double speedFactor = fastRotation ? 1.0 : (1 - msg->axes[Xbox::rightTriggerAxis]) / 2;
        const Vector2 motorValues = GetMotorValues(moveVector, static_cast<float>(speedFactor), fastRotation);

        if (!CommonHelper::approximatelyEqual(driveMsg_.left_speed, motorValues.x) || !CommonHelper::approximatelyEqual(driveMsg_.right_speed, motorValues.y)) {
            const auto now = node->get_clock()->now();
            if (!lastProcessTime_.has_value() || now - *lastProcessTime_ > minProcessPublishInterval_) {
                driveMsg_.left_speed = motorValues.x;
                driveMsg_.right_speed = motorValues.y;
                if (publishCallback_) {
                    publishCallback_(driveMsg_);
                }
                lastProcessTime_ = now;
            }
        }
    }
};

#endif //DRIVE_MANAGER_H
