#include "UART.hpp"
#include <stdio.h>
#include <string.h>

Exodus::UART::UART() : initialized(false) {}

Exodus::UART::UART(UARTOptions option, unsigned int baud, unsigned int config, bool interruptEnable) {
    this->initialize(option, baud, config, interruptEnable);
}

void Exodus::UART::initialize(UARTOptions option, unsigned int baud, unsigned int config, bool interruptEnable) {
    uint32_t module, port, portBase, pinRX, pinTX, intModule;
    uint8_t pinMask;

    this -> baud = baud;

    // For a certain module, there are defined input macros for the following functions
    // this switch table helps the object load the correct macros into the TivaWare Drivers
    switch (option) {
        case UART0A:
            module = SYSCTL_PERIPH_UART0;
            port = SYSCTL_PERIPH_GPIOA;
            portBase = GPIO_PORTA_BASE;
            moduleBase = UART0_BASE;
            pinRX = GPIO_PA0_U0RX;
            pinTX = GPIO_PA1_U0TX;
            pinMask = GPIO_PIN_0 | GPIO_PIN_1;
            intModule = INT_UART0;
            break;
        case UART1B:
            module = SYSCTL_PERIPH_UART1;
            port = SYSCTL_PERIPH_GPIOB;
            portBase = GPIO_PORTB_BASE;
            moduleBase = UART1_BASE;
            pinRX = GPIO_PB0_U1RX;
            pinTX = GPIO_PB1_U1TX;
            pinMask = GPIO_PIN_0 | GPIO_PIN_1;
            intModule = INT_UART1;
            break;
        case UART1C:
            module = SYSCTL_PERIPH_UART1;
            port = SYSCTL_PERIPH_GPIOC;
            portBase = GPIO_PORTC_BASE;
            moduleBase = UART1_BASE;
            pinRX = GPIO_PC4_U1RX;
            pinTX = GPIO_PC5_U1TX;
            pinMask = GPIO_PIN_4 | GPIO_PIN_5;
            intModule = INT_UART1;
            break;
        case UART2D:
            module = SYSCTL_PERIPH_UART2;
            port = SYSCTL_PERIPH_GPIOD;
            portBase = GPIO_PORTD_BASE;
            moduleBase = UART2_BASE;
            pinRX = GPIO_PD6_U2RX;
            pinTX = GPIO_PD7_U2TX;
            pinMask = GPIO_PIN_6 | GPIO_PIN_7;
            intModule = INT_UART2;
            break;
        case UART3C:
            module = SYSCTL_PERIPH_UART3;
            port = SYSCTL_PERIPH_GPIOC;
            portBase = GPIO_PORTC_BASE;
            moduleBase = UART3_BASE;
            pinRX = GPIO_PC6_U3RX;
            pinTX = GPIO_PC7_U3TX;
            pinMask = GPIO_PIN_6 | GPIO_PIN_7;
            intModule = INT_UART3;
            break;
        case UART4C:
            module = SYSCTL_PERIPH_UART4;
            port = SYSCTL_PERIPH_GPIOC;
            portBase = GPIO_PORTC_BASE;
            moduleBase = UART4_BASE;
            pinRX = GPIO_PC4_U4RX;
            pinTX = GPIO_PC5_U4TX;
            pinMask = GPIO_PIN_4 | GPIO_PIN_5;
            intModule = INT_UART4;
            break;
        case UART5E:
            module = SYSCTL_PERIPH_UART5;
            port = SYSCTL_PERIPH_GPIOE;
            portBase = GPIO_PORTE_BASE;
            moduleBase = UART5_BASE;
            pinRX = GPIO_PE4_U5RX;
            pinTX = GPIO_PE5_U5TX;
            pinMask = GPIO_PIN_4 | GPIO_PIN_5;
            intModule = INT_UART5;
            break;
        case UART6D:
            module = SYSCTL_PERIPH_UART6;
            port = SYSCTL_PERIPH_GPIOD;
            portBase = GPIO_PORTD_BASE;
            moduleBase = UART6_BASE;
            pinRX = GPIO_PD4_U6RX;
            pinTX = GPIO_PD5_U6TX;
            pinMask = GPIO_PIN_4 | GPIO_PIN_5;
            intModule = INT_UART6;
            break;
        case UART7E:
            module = SYSCTL_PERIPH_UART7;
            port = SYSCTL_PERIPH_GPIOE;
            portBase = GPIO_PORTE_BASE;
            moduleBase = UART7_BASE;
            pinRX = GPIO_PE0_U7RX;
            pinTX = GPIO_PE1_U7TX;
            pinMask = GPIO_PIN_0 | GPIO_PIN_1;
            intModule = INT_UART7;
            break;
    }

    SysCtlPeripheralEnable(module);
    SysCtlPeripheralEnable(port);

    GPIOPinConfigure(pinRX);
    GPIOPinConfigure(pinTX);

    GPIOPinTypeUART(portBase, pinMask);

    UARTConfigSetExpClk(moduleBase, SysCtlClockGet(), baud, config);

    if (interruptEnable) {
        IntMasterEnable(); // enable processor interrupts
        IntEnable(intModule);
        UARTIntEnable(moduleBase, UART_INT_RX | UART_INT_RT);  // only enable RX and TX interrupts
    }

    for (int i = 0; i < 200; ++i) buffer[i] = '\0';

    initialized = true;
}

unsigned int Exodus::UART::getModuleBase() {
    return moduleBase;
}


//******************************************
//
//          Ported UARTSTDIO.C Code
//
//******************************************
int Exodus::UART::write(const char *pcBuf, uint32_t ui32Len) {
    unsigned int uIdx;

    // Send the characters
    for(uIdx = 0; uIdx < ui32Len; uIdx++) {
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output.
        if(pcBuf[uIdx] == '\n') {
            MAP_UARTCharPut(moduleBase, '\r');
        }

        // Send the character to the UART output.
        MAP_UARTCharPut(moduleBase, pcBuf[uIdx]);
        buffer[uIdx] = '\0';
    }

    // Return the number of characters written.
    return uIdx;
}

void Exodus::UART::printf(const char *pcString, ...) {
    va_list args;
    va_start(args, pcString);
    int length = vsprintf(buffer, pcString, args);
    va_end(args);
    write(buffer, length);

}

