#ifndef SCREEN_ANIMATION_MANAGER_H
#define SCREEN_ANIMATION_MANAGER_H

#include "types.h"
#include "mimi_interfaces/msg/scr_anim_cmd.hpp"

using PublishScrAnimCmdCallback = std::function<void(mimi_interfaces::msg::ScrAnimCmd& scrAnimMsg)>;

class ScreenAnimationManager {
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

    PublishScrAnimCmdCallback publishCallback_ = nullptr;

    mimi_interfaces::msg::ScrAnimCmd scrAnimMsg_;
public:
    void setPublishCallback(PublishScrAnimCmdCallback publishCallback) {
        publishCallback_ = std::move(publishCallback);
    }

    void process(const sensor_msgs::msg::Joy::UniquePtr &msg) {
        const bool bumperPressed = msg->buttons[Xbox::leftBumperButton] == 1;
        const bool pressedA = msg->buttons[Xbox::buttonA] == 1;
        const bool pressedB = msg->buttons[Xbox::buttonB] == 1;
        const bool pressedX = msg->buttons[Xbox::buttonX] == 1;
        const bool pressedY = msg->buttons[Xbox::buttonY] == 1;
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
            if (publishCallback_) {
                publishCallback_(scrAnimMsg_);
            }
        }
    }
};

#endif //SCREEN_ANIMATION_MANAGER_H
