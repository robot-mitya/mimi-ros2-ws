#ifndef CONTROL_MODE_MANAGER_H
#define CONTROL_MODE_MANAGER_H

#include "types.h"

enum class ControlMode {
    ScreenAnimations,
    Headlights,
};

using ControlModeChangedCallback = std::function<void(const ControlMode& controlMode)>;

class ControlModeManager {
    ControlMode controlMode_ = ControlMode::ScreenAnimations;
    ControlModeChangedCallback controlModeChangedCallback_ = nullptr;
public:
    ControlMode getControlMode() const { return controlMode_; }

    void setControlModeChangedCallback(const ControlModeChangedCallback controlModeChangedCallback) {
        controlModeChangedCallback_ = std::move(controlModeChangedCallback);
    }

    void process(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        if (CommonHelper::approximatelyEqual(msg->axes[Xbox::dPadAxisX], -1)) { // D-PAD RIGHT
            if (controlMode_ != ControlMode::Headlights) {
                controlMode_ = ControlMode::Headlights;
                if (controlModeChangedCallback_) {
                    controlModeChangedCallback_(controlMode_);
                }
            }
        } else if (CommonHelper::approximatelyEqual(msg->axes[Xbox::dPadAxisY], 1)) { // D-PAD UP
            if (controlMode_ != ControlMode::ScreenAnimations) {
                controlMode_ = ControlMode::ScreenAnimations;
                if (controlModeChangedCallback_) {
                    controlModeChangedCallback_(controlMode_);
                }
            }
        }
    }
};

#endif //CONTROL_MODE_MANAGER_H
