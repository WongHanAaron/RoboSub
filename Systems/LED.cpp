/*
 * LED.cpp
 *
 *  Created on: Jun 1, 2020
 *      Author: Loren
 */

#include "LED.hpp"

bool Exodus::LED::redInitialized = false;
bool Exodus::LED::blueInitialized = false;
bool Exodus::LED::greenInitialized = false;

Exodus::LED::LED() {}

int Exodus::LED::initialize(led_option option, JetsonLink* tilink) {
    switch(option) {
        case RED:
            if (redInitialized) {
                if (tilink != 0) tilink -> printf("RED LED is already initialized\n~");
                return 0;
            }
            red.initialize(PF1M1G2);
            redInitialized = true;
        break;
        case GREEN:
            if (greenInitialized) {
                if (tilink != 0) tilink -> printf("GREEN LED is already initialized\n~");
                return 0;
            }
            green.initialize(PF3M1G3);
            greenInitialized = true;
        break;
        case BLUE:
            if (blueInitialized) {
                if (tilink != 0) tilink -> printf("BLUE LED is already initialized\n~");
                return 0;
            }
            blue.initialize(PF2M1G3);
            blueInitialized = true;
        break;
    }

    return 1;
}

int Exodus::LED::initializeAll() {
    return (initialize(RED) and initialize(BLUE) and initialize(GREEN));
}

void Exodus::LED::toggle(led_option option) {
    switch(option) {
        case RED:
            redInitialized = !redInitialized;
            red.setOutput(redInitialized);
        break;
        case GREEN:
            greenInitialized = !greenInitialized;
            green.setOutput(greenInitialized);
        break;
        case BLUE:
            blueInitialized = !blueInitialized;
            blue.setOutput(blueInitialized);
        break;
    }
}

void Exodus::LED::toggleAll() {
    toggle(RED);
    toggle(BLUE);
    toggle(GREEN);
}

void Exodus::LED::setDutyCycle(led_option option, double percentage) {
    switch(option) {
        case RED:
            red.setDutyCycle(percentage);
        break;
        case BLUE:
            blue.setDutyCycle(percentage);
        break;
        case GREEN:
            green.setDutyCycle(percentage);
        break;
    }
}

void Exodus::LED::setAllDutyCycle(double percentage) {
    setDutyCycle(RED, percentage);
    setDutyCycle(BLUE, percentage);
    setDutyCycle(GREEN, percentage);
}
