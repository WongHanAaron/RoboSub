/*
 * Servo.cpp
 *
 *  Created on: Jun 1, 2020
 *      Author: Loren
 */

#include "Servo.hpp"

void Exodus::Servo::initialize(PWMPinOptions option) {
    servoPin.initialize(option);
}

void Exodus::Servo::rotate(uint32_t degrees) {
    if (degrees > 180)
        degrees = 180;

    position = degrees;

    uint32_t width = (uint32_t)(servoPin.getPeriod() * position/180.0);

    servoPin.setWidth(width);
}

void Exodus::Servo::setPeriod(uint32_t period) {
    servoPin.setPeriod(period);
}

uint32_t Exodus::Servo::getWidth() {
    return servoPin.getWidth();
}

void Exodus::Servo::setWidth(uint32_t width) {
    servoPin.setWidth(width);
}
