#ifndef TYPES_H
#define TYPES_H

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
    [[nodiscard]] bool isSame(const float rr, const float gg, const float bb) const {
        return r == rr && g == gg && b == bb;
    }
    [[nodiscard]] bool isSame(const Color3 other) const {
        return r == other.r && g == other.g && b == other.b;
    }
};

class Xbox {
public:
    static constexpr int leftJoystickAxisX = 0;
    static constexpr int leftJoystickAxisY = 1;
    static constexpr int leftTriggerAxis = 2;
    static constexpr int rightJoystickAxisX = 3;
    static constexpr int rightJoystickAxisY = 4;
    static constexpr int rightTriggerAxis = 5;
    static constexpr int dPadAxisX = 6;
    static constexpr int dPadAxisY = 7;
    static constexpr int buttonA = 0;
    static constexpr int buttonB = 1;
    static constexpr int buttonX = 2;
    static constexpr int buttonY = 3;
    static constexpr int leftBumperButton = 4;
    static constexpr int rightBumperButton = 5;
    static constexpr int backButton = 6;
    static constexpr int startButton = 7;
    static constexpr int guideButton = 8;
    static constexpr int leftStickButton = 9;
    static constexpr int rightStickButton = 10;
};

class CommonHelper {
public:
    static bool approximatelyEqual(const float a, const float b) {
        return std::fabs(a - b) < 1e-3f;
    }

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
};

#endif //TYPES_H
