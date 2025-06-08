//
// Created by dmitrydzz on 6/8/25.
//

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

struct Vector2 {
    double x;
    double y;

    Vector2() : x(0), y(0) {
    }

    Vector2(const double _x, const double _y) : x(_x), y(_y) {
    }
};

class GamepadNode final : public rclcpp::Node {
    static constexpr double maxSpeed = 1.0;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

    static Vector2 CircleToSquareInFirstQuadrant(Vector2 value) {
        double x = value.x;
        double y = value.y;
        if (x == 0) return value; // (to avoid dividing by 0)
        if (x >= 0 && y >= 0) {
            const bool firstOctantInQuadrant = x >= y;
            if (!firstOctantInQuadrant) std::swap(x, y);
            const double resultX = sqrt(x * x + y * y);
            const double resultY = y * resultX / x;
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
        value.x = std::clamp(value.x, -1.0, 1.0);
        value.y = std::clamp(value.y, -1.0, 1.0);
        return value;
    }

    static Vector2 GetMotorValues(const Vector2 moveVector, const double speedFactor, const bool fastRotation) {
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
        return Vector2(leftSpeed, rightSpeed);
    }

public:
    GamepadNode()
        : Node("gamepad_node") {
        auto joyTopicCallback =
                [this](const sensor_msgs::msg::Joy::UniquePtr &msg) -> void {
            const Vector2 joystickVector(-msg->axes[0], msg->axes[1]);          // Left joystick
            const Vector2 moveVector = CircleToSquare(joystickVector);
            const bool fastRotation = msg->buttons[5] > 0;                          // Right shoulder
            const double speedFactor = fastRotation ? 1.0 : (1 - msg->axes[5]) / 2; // Right trigger
            const Vector2 motorValues = GetMotorValues(moveVector, speedFactor, fastRotation);
            // RCLCPP_INFO(this->get_logger(), "Left joystick: [%4.1f, %4.1f], speed: [%4.1f]", msg->axes[0], msg->axes[1], msg->axes[5]);
            // RCLCPP_INFO(this->get_logger(), "Move vector: [%4.1f, %4.1f], speedFactor: [%4.1f], fastRotation: [%d]", moveVector.x, moveVector.y, speedFactor, fastRotation);
            RCLCPP_INFO(this->get_logger(), "Motor values: [%4.1f, %4.1f]", motorValues.x, motorValues.y);
        };
        subscription_ =
                this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, joyTopicCallback);
    }
};

int main(const int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GamepadNode>());
    rclcpp::shutdown();
    return 0;
}
