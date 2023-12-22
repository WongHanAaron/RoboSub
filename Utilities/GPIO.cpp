#include "GPIO.hpp"


void Exodus::GPIO::initialize(pin option, direction pinDirection) {
    this -> isOutput = pinDirection;

    uint32_t port = (option <= PA7) ? SYSCTL_PERIPH_GPIOA :
                    (option <= PB7) ? SYSCTL_PERIPH_GPIOB :
                    (option <= PC7) ? SYSCTL_PERIPH_GPIOC :
                    (option <= PD7) ? SYSCTL_PERIPH_GPIOD :
                    (option <= PE5) ? SYSCTL_PERIPH_GPIOE :
                    (option <= PF4) ? SYSCTL_PERIPH_GPIOF :
                                     0                   ;
    /*Check if valid pin is */
    if (port == 0) return;

    portBase = (option <= PA7) ? GPIO_PORTA_BASE :
               (option <= PB7) ? GPIO_PORTB_BASE :
               (option <= PC7) ? GPIO_PORTC_BASE :
               (option <= PD7) ? GPIO_PORTD_BASE :
               (option <= PE5) ? GPIO_PORTE_BASE :
               (option <= PF4) ? GPIO_PORTF_BASE :
                                 0               ;

    pinMask = (option % 8 == 0) ? GPIO_PIN_0 :
              (option % 8 == 1) ? GPIO_PIN_1 :
              (option % 8 == 2) ? GPIO_PIN_2 :
              (option % 8 == 3) ? GPIO_PIN_3 :
              (option % 8 == 4) ? GPIO_PIN_4 :
              (option % 8 == 5) ? GPIO_PIN_5 :
              (option % 8 == 6) ? GPIO_PIN_6 :
                                  GPIO_PIN_7 ;

    SysCtlPeripheralEnable(port);
    while(!SysCtlPeripheralReady(port));

    if (this -> isOutput)
        GPIOPinTypeGPIOOutput(portBase, pinMask);
    else
        GPIOPinTypeGPIOInput(portBase, pinMask);
}

void Exodus::GPIO::setAsOutput() {
    isOutput = true;
    GPIOPinTypeGPIOOutput(portBase, pinMask);
}

void Exodus::GPIO::setAsInput() {
    isOutput = false;
    GPIOPinTypeGPIOInput(portBase, pinMask);
}

bool Exodus::GPIO::isOutputMode() {
    return isOutput;
}

int Exodus::GPIO::write(gpio_state state) {
    if (!isOutput) return -1;

    if (state == high)
        GPIOPinWrite(portBase, pinMask, pinMask);
    else if (state == low)
        GPIOPinWrite(portBase, pinMask, 0);
    else
        return -1;

    return 0;
}

int Exodus::GPIO::read() {
    if (isOutput) return -1;

    uint32_t readValue = GPIOPinRead(portBase, pinMask) & pinMask;
    if (readValue == 0) return low;

    return high;
}
