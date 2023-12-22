/**
 * The UART class acts as a means of interracting with other devices via UART communication.
 * This means of communicating uses two pins (a receive and transmit pin) on TWO devices at a time.
 * They synchronize the communication between communicating at specified rate (baud rate).
 */
#include "Definitions.hpp"
#include <stdint.h>
#include <stdarg.h>
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"


#ifndef UART_HPP
#define UART_HPP

namespace Exodus
{

#define UART_RX_BUFFER_SIZE     128
#define UART_TX_BUFFER_SIZE     1024


class UART {
public:
    UART();
    UART(UARTOptions option, uint32_t baud, unsigned int config, bool interruptEnable = false);
    /**
     * Initialize UART pins
     * @params: UART communication option to determine which pins will e used (option)
     *          Baudrate of UART (baud)
     *          Configuration option of UART (config)
     *          Option of whether or not to set a corresponding interrupt (interruptEnable)
     * @return: none
     */
    void initialize(UARTOptions option, uint32_t baud, unsigned int config, bool interruptEnable = false);

    /**
     * Print string to Mission Control GUI
     * The use of this function is similar to that of printf in the stdio.h library
     * @params: string to print (pcString)
     *          arguments of corresponding string (...)
     * @return: none
     */
    void printf(const char *pcString, ...);

    /**
     * Get the module base of the UART communication
     * @params: none
     * @return: UART module base
     */
    uint32_t getModuleBase();
protected:
    uint32_t moduleBase;
    uint32_t baud;
    char buffer[200];
    bool initialized;

    /**
     * Writes the string to a device connected via UART
     * @params: string to write (pcBuf)
     *          length of string (ui32Len)
     * @return: number of characters written
     */
    int write(const char *pcBuf, uint32_t ui32Len);

};

}

#endif /* UART_HPP */
