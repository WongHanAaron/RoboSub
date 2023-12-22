/*
 * MiniBot.cpp
 *
 *  Created on: January 31, 2020
 *  Author: Loren
 */

#include "MiniBot.hpp"

void Exodus::MiniBot::initialize(JetsonLink *tilink, PWMPinOptions rightForward, PWMPinOptions rightBackward, PWMPinOptions leftForward, PWMPinOptions leftBackward) {
    rightMotorForward.initialize(rightForward);
    rightMotorBackward.initialize(rightBackward);
    leftMotorForward.initialize(leftForward);
    leftMotorBackward.initialize(leftBackward);

    rightMotorForward.setWidth((uint32_t)1);
    rightMotorBackward.setWidth((uint32_t)1);
    leftMotorForward.setWidth((uint32_t)1);
    leftMotorBackward.setWidth((uint32_t)1);

    rightWidth = 1500;
    leftWidth = 1500;

    tilink -> printf("Initialized MiniBot Motors~\n");
    link = tilink;

}

uint32_t Exodus::MiniBot::calculateWidth(double percentage) {
    percentage = (percentage > 100) ? 100       :
                 (percentage < 0  ) ? 0         :
                                      percentage;

    return (uint32_t)(MINIBOT_RANGE * percentage/100.0);
}
void Exodus::MiniBot::setRightWidth(uint32_t width) {
        int value = (int)width - 1500;
//        rightWidth = 5 * std::abs(value) + 500;
        rightWidth = MINIBOT_MIN_WIDTH + MINIBOT_RANGE * std::abs(value) / SUB_MOTOR_RANGE; // This is a conversion to mimic the sub operation
        if (rightWidth > 2000) rightWidth = 2000;

        if (std::abs(value) < 30){
            // Right Motor is not moving
            rightMotorForward.setWidth((uint32_t)1);
            rightMotorBackward.setWidth((uint32_t)1);
        } else if (value > 0) {
            // Right Motor is moving forward
            rightMotorForward.setWidth(rightWidth);
            rightMotorBackward.setWidth((uint32_t)1);
        } else {
            // Right Motor is moving backward
            rightMotorForward.setWidth((uint32_t)1);
            rightMotorBackward.setWidth(rightWidth);
        }
        rightWidth = width;

}

void Exodus::MiniBot::setRightWidth(bool isForward, double percentage) {
    rightWidth = calculateWidth(percentage);

    if (rightWidth < MINIBOT_DEADBAND_DUTY) {
        rightWidth = 1500;
        rightMotorForward.setWidth(0.0);
        rightMotorBackward.setWidth(0.0);
    } else if (isForward) {
        rightMotorForward.setWidth(MINIBOT_MIN_WIDTH + rightWidth);
        rightMotorBackward.setWidth(0.0);
        rightWidth = 1500 + (uint32_t)(300 * percentage/100.0);
    } else {
        rightMotorBackward.setWidth(MINIBOT_MIN_WIDTH + rightWidth);
        rightMotorForward.setWidth(0.0);
        rightWidth = 1500 - (uint32_t)(300 * percentage/100.0);
    }
}

void Exodus::MiniBot::setLeftWidth(uint32_t width) {
        int value = (int)width - 1500;
//        leftWidth = 5 * std::abs(value) + 500;
        leftWidth = MINIBOT_MIN_WIDTH + MINIBOT_RANGE * std::abs(value) / SUB_MOTOR_RANGE; // This is a conversion to mimic the sub operation
        if (leftWidth > 2000) leftWidth = 2000;

        if (std::abs(value) < SUB_MOTOR_DEADBAND_HALF){
            // Left Motor is not moving
            leftMotorForward.setWidth((uint32_t)1);
            leftMotorBackward.setWidth((uint32_t)1);
        } else if (value > 0) {
            // Left Motor is moving forward
            leftMotorForward.setWidth(leftWidth);
            leftMotorBackward.setWidth((uint32_t)1);
        } else {
            // Left Motor is moving backward
            leftMotorForward.setWidth((uint32_t)1);
            leftMotorBackward.setWidth(leftWidth);
        }
        leftWidth = width;
}

void Exodus::MiniBot::setLeftWidth(bool isForward, double percentage) {
    leftWidth = calculateWidth(percentage);

    if (leftWidth < MINIBOT_DEADBAND_DUTY) {
        leftWidth = 1500;
        leftMotorForward.setWidth(0.0);
        leftMotorBackward.setWidth(0.0);
    } else if (isForward) {
        leftMotorForward.setWidth(MINIBOT_MIN_WIDTH + leftWidth);
        leftMotorBackward.setWidth(0.0);
        leftWidth = 1500 + (uint32_t)(300 * percentage/100.0);
    } else {
        leftMotorBackward.setWidth(MINIBOT_MIN_WIDTH + leftWidth);
        leftMotorForward.setWidth(0.0);
        rightWidth = 1500 - (uint32_t)(300 * percentage/100.0);
    }
}

uint32_t Exodus::MiniBot::getRightWidth() {
    return rightWidth;
}

uint32_t Exodus::MiniBot::getLeftWidth() {
    return leftWidth;
}
