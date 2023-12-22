#include "PWM.hpp"
#include "driverlib/sysctl.h"

Exodus::PWMPin::PWMPin() {}

void Exodus::PWMPin::initialize(PWMPinOptions option, float conversionRatio, unsigned int width, unsigned int period, unsigned int config, unsigned int clock) {
    uint32_t port;
    uint32_t module;
    uint32_t pinGPIO;
    uint32_t portBase;
    uint32_t pinMask;

    ConversionRatio = conversionRatio;
    switch(option) {
        case PB6M0G0:
            port = SYSCTL_PERIPH_GPIOB;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PB6_M0PWM0;
            portBase = GPIO_PORTB_BASE;
            pinMask = GPIO_PIN_6;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_0;
            pwmOut = PWM_OUT_0;
            pwmOutBit = PWM_OUT_0_BIT;
            break;
        case PB7M0G0:
            port = SYSCTL_PERIPH_GPIOB;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PB7_M0PWM1;
            portBase = GPIO_PORTB_BASE;
            pinMask = GPIO_PIN_7;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_0;
            pwmOut = PWM_OUT_1;
            pwmOutBit = PWM_OUT_1_BIT;
            break;
        case PB4M0G1:
            port = SYSCTL_PERIPH_GPIOB;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PB4_M0PWM2;
            portBase = GPIO_PORTB_BASE;
            pinMask = GPIO_PIN_4;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_1;
            pwmOut = PWM_OUT_2;
            pwmOutBit = PWM_OUT_2_BIT;
            break;
        case PB5M0G1:
            port = SYSCTL_PERIPH_GPIOB;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PB5_M0PWM3;
            portBase = GPIO_PORTB_BASE;
            pinMask = GPIO_PIN_5;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_1;
            pwmOut = PWM_OUT_3;
            pwmOutBit = PWM_OUT_3_BIT;
            break;
        case PE4M0G2:
            port = SYSCTL_PERIPH_GPIOE;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PE4_M0PWM4;
            portBase = GPIO_PORTE_BASE;
            pinMask = GPIO_PIN_4;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_2;
            pwmOut = PWM_OUT_4;
            pwmOutBit = PWM_OUT_4_BIT;
            break;
        case PE5M0G2:
            port = SYSCTL_PERIPH_GPIOE;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PE5_M0PWM5;
            portBase = GPIO_PORTE_BASE;
            pinMask = GPIO_PIN_5;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_2;
            pwmOut = PWM_OUT_5;
            pwmOutBit = PWM_OUT_5_BIT;
            break;
        case PC4M0G3:
            port = SYSCTL_PERIPH_GPIOC;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PC4_M0PWM6;
            portBase = GPIO_PORTC_BASE;
            pinMask = GPIO_PIN_4;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_3;
            pwmOut = PWM_OUT_6;
            pwmOutBit = PWM_OUT_6_BIT;
            break;
        case PD0M0G3:
            port = SYSCTL_PERIPH_GPIOD;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PD0_M0PWM6;
            portBase = GPIO_PORTD_BASE;
            pinMask = GPIO_PIN_0;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_3;
            pwmOut = PWM_OUT_6;
            pwmOutBit = PWM_OUT_6_BIT;
            break;
        case PC5M0G3:
            port = SYSCTL_PERIPH_GPIOC;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PC5_M0PWM7;
            portBase = GPIO_PORTC_BASE;
            pinMask = GPIO_PIN_5;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_3;
            pwmOut = PWM_OUT_7;
            pwmOutBit = PWM_OUT_7_BIT;
            break;
        case PD1M0G3:
            port = SYSCTL_PERIPH_GPIOD;
            module = SYSCTL_PERIPH_PWM0;
            pinGPIO = GPIO_PD1_M0PWM7;
            portBase = GPIO_PORTD_BASE;
            pinMask = GPIO_PIN_1;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_3;
            pwmOut = PWM_OUT_7;
            pwmOutBit = PWM_OUT_7_BIT;
            break;
        case PD0M1G0:
            port = SYSCTL_PERIPH_GPIOD;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PD0_M1PWM0;
            portBase = GPIO_PORTD_BASE;
            pinMask = GPIO_PIN_0;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_0;
            pwmOut = PWM_OUT_0;
            pwmOutBit = PWM_OUT_0_BIT;
            break;
        case PD1M1G0:
            port = SYSCTL_PERIPH_GPIOD;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PD1_M1PWM1;
            portBase = GPIO_PORTD_BASE;
            pinMask = GPIO_PIN_1;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_0;
            pwmOut = PWM_OUT_1;
            pwmOutBit = PWM_OUT_1_BIT;
            break;
        case PA6M1G1:   // camera servo
            port = SYSCTL_PERIPH_GPIOA;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PA6_M1PWM2;
            portBase = GPIO_PORTA_BASE;
            pinMask = GPIO_PIN_6;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_1;
            pwmOut = PWM_OUT_2;
            pwmOutBit = PWM_OUT_2_BIT;
            break;
        case PE4M1G1:
            port = SYSCTL_PERIPH_GPIOE;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PE4_M1PWM2;
            portBase = GPIO_PORTE_BASE;
            pinMask = GPIO_PIN_4;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_1;
            pwmOut = PWM_OUT_2;
            pwmOutBit = PWM_OUT_2_BIT;
            break;
        case PA7M1G1:   // used for lights
            port = SYSCTL_PERIPH_GPIOA;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PA7_M1PWM3;
            portBase = GPIO_PORTA_BASE;
            pinMask = GPIO_PIN_7;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_1;
            pwmOut = PWM_OUT_3;
            pwmOutBit = PWM_OUT_3_BIT;
            break;
        case PE5M1G1:
            port = SYSCTL_PERIPH_GPIOE;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PE5_M1PWM3;
            portBase = GPIO_PORTE_BASE;
            pinMask = GPIO_PIN_5;
            pwmBase = PWM0_BASE;
            pwmGen = PWM_GEN_1;
            pwmOut = PWM_OUT_3;
            pwmOutBit = PWM_OUT_3_BIT;
            break;
        case PF0M1G2:
            port = SYSCTL_PERIPH_GPIOF;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PF0_M1PWM4;
            portBase = GPIO_PORTF_BASE;
            pinMask = GPIO_PIN_0;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_2;
            pwmOut = PWM_OUT_4;
            pwmOutBit = PWM_OUT_4_BIT;
            break;
        case PF1M1G2:
            port = SYSCTL_PERIPH_GPIOF;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PF1_M1PWM5;
            portBase = GPIO_PORTF_BASE;
            pinMask = GPIO_PIN_1;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_2;
            pwmOut = PWM_OUT_5;
            pwmOutBit = PWM_OUT_5_BIT;
            break;
        case PF2M1G3:
            port = SYSCTL_PERIPH_GPIOF;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PF2_M1PWM6;
            portBase = GPIO_PORTF_BASE;
            pinMask = GPIO_PIN_2;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_3;
            pwmOut = PWM_OUT_6;
            pwmOutBit = PWM_OUT_6_BIT;
            break;
        case PF3M1G3:
            port = SYSCTL_PERIPH_GPIOF;
            module = SYSCTL_PERIPH_PWM1;
            pinGPIO = GPIO_PF3_M1PWM7;
            portBase = GPIO_PORTF_BASE;
            pinMask = GPIO_PIN_3;
            pwmBase = PWM1_BASE;
            pwmGen = PWM_GEN_3;
            pwmOut = PWM_OUT_7;
            pwmOutBit = PWM_OUT_7_BIT;
            break;
        default:
            return;
    }
    this -> width = width;
    SysCtlPWMClockSet(clock);
    SysCtlPeripheralEnable(port);
    SysCtlPeripheralEnable(module);
    GPIOPinConfigure(pinGPIO);
    GPIOPinTypePWM(portBase, pinMask);
    PWMGenConfigure(pwmBase, pwmGen, config);
    PWMGenPeriodSet(pwmBase, pwmGen, period*ConversionRatio);
    PWMPulseWidthSet(pwmBase, pwmOut, width*ConversionRatio);
    PWMGenEnable(pwmBase, pwmGen);
    PWMOutputState(pwmBase, pwmOutBit, true);
}


void Exodus::PWMPin::setPeriod(uint32_t period, float conversionRatio) { // allows you to set a new conversion ratio
    PWMGenPeriodSet(pwmBase, pwmGen, period*conversionRatio);
}

void Exodus::PWMPin::setPeriod(uint32_t period) {    // overloaded function uses the object's saved conversion ratio
    setPeriod(period, ConversionRatio);
}

void Exodus::PWMPin::setWidth(uint32_t width, float conversionRatio) { // allows you to set a new conversion ratio
    this -> width = width;
    PWMPulseWidthSet(pwmBase, pwmOut, width*conversionRatio);
}

void Exodus::PWMPin::setDutyCycle(double percentage) {   // overloaded function sets width based on percentage
    uint32_t width;
//    width = PWMGenPeriodGet(pwmBase, pwmGen) * (percentage / 100); // duty cycle based on percentage
    width = 2000 * percentage / 100;
    setWidth(width, ConversionRatio);
}

uint32_t Exodus::PWMPin::getPeriod() {
   return PWMGenPeriodGet(pwmBase, pwmGen);
}

void Exodus::PWMPin::setWidth(uint32_t width) { // overloaded function sets width
    setWidth(width, ConversionRatio);
}

void Exodus::PWMPin::setOutput(bool enabled) {
    PWMOutputState(pwmBase, pwmOutBit, enabled);
}

uint32_t Exodus::PWMPin::getWidth() {
    return width;
}
