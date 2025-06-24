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

#endif //TYPES_H
