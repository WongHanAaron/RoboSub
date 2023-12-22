#include "Motor.hpp"


void Exodus::Motor::initialize(PWMPinOptions leftVertical, PWMPinOptions rightVertical, PWMPinOptions leftHorizontal, PWMPinOptions rightHorizontal) {
    LeftVertical.initialize(leftVertical);
    RightVertical.initialize(rightVertical);
    LeftHorizontal.initialize(leftHorizontal);
    RightHorizontal.initialize(rightHorizontal);

    LeftHorizontal.setWidth((uint32_t)1500);
    RightHorizontal.setWidth((uint32_t)1500);
    LeftVertical.setWidth((uint32_t)1500);
    RightVertical.setWidth((uint32_t)1500);
}

void Exodus::Motor::setLeftHorizontalWidth(bool isForward, double percentage) {
    percentage = (percentage > 100) ? 100       :
                 (percentage < 0  ) ? 0         :
                                      percentage;
    uint32_t width = (uint32_t)(SUB_MOTOR_RANGE * percentage/100.0);

    if (isForward)
        LeftHorizontal.setWidth(SUB_MOTOR_ZERO + width);
    else
        LeftHorizontal.setWidth(SUB_MOTOR_ZERO - width);
}

uint32_t Exodus::Motor::calculateWidth(double percentage) {
    percentage = (percentage > 100) ? 100       :
                 (percentage < 0  ) ? 0         :
                                      percentage;

    return (uint32_t)(SUB_MOTOR_RANGE * percentage/100.0);

}

void Exodus::Motor::setRightHorizontalWidth(bool isForward, double percentage) {
    if (isForward)
        RightHorizontal.setWidth(SUB_MOTOR_ZERO + calculateWidth(percentage));
    else
        RightHorizontal.setWidth(SUB_MOTOR_ZERO - calculateWidth(percentage));
}

void Exodus::Motor::setLeftVerticalWidth(bool isForward, double percentage) {
    if (isForward)
        LeftVertical.setWidth(SUB_MOTOR_ZERO + calculateWidth(percentage));
    else
        LeftVertical.setWidth(SUB_MOTOR_ZERO - calculateWidth(percentage));
}

void Exodus::Motor::setRightVerticalWidth(bool isForward, double percentage) {
    if (isForward)
        RightVertical.setWidth(SUB_MOTOR_ZERO + calculateWidth(percentage));
    else
        RightVertical.setWidth(SUB_MOTOR_ZERO - calculateWidth(percentage));
}

uint32_t Exodus::Motor::getLeftHorizontalWidth() {
    return LeftHorizontal.getWidth();
}

uint32_t Exodus::Motor::getRightHorizontalWidth() {
    return RightHorizontal.getWidth();
}

uint32_t Exodus::Motor::getLeftVerticalWidth() {
    return LeftVertical.getWidth();
}

uint32_t Exodus::Motor::getRightVerticalWidth() {
    return RightVertical.getWidth();
}

void Exodus::Motor::setLeftHorizontalWidth(uint32_t width) {
    LeftHorizontal.setWidth(width);
}

void Exodus::Motor::setRightHorizontalWidth(uint32_t width) {
    RightHorizontal.setWidth(width);
}

void Exodus::Motor::setLeftVerticalWidth(uint32_t width) {
    LeftVertical.setWidth(width);
}

void Exodus::Motor::setRightVerticalWidth(uint32_t width) {
    RightVertical.setWidth(width);
}

void Exodus::Motor::setAllWidths(uint32_t width) {
    LeftHorizontal.setWidth(width);
    RightHorizontal.setWidth(width);
    LeftVertical.setWidth(width);
    RightVertical.setWidth(width);
}
