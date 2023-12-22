/**
 * GPIOPin class is used to operate the pins of the TM4C123 micro-controller
 * These pins serve either input or output functionality where they
 * read/write binary states consisting of HIGH (1) or LOW (0).
 */

#include "Definitions.hpp"
#include "stdint.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "JetsonLink.hpp"

#ifndef GPIO_HPP
#define GPIO_HPP


namespace Exodus {

class GPIO {
public:
    /**
     * Initialize GPIO pin
     * @params: pin to initialize (option)
     *          direction of the pin (pinDirection)
     * @return: none
     */
    void initialize(pin option, direction pinDirection);

    /**
     * Set the pin to become an output pin
     * @params: none
     * @return: none
     */
    void setAsOutput();

    /**
     * Set the pin to become an input pin
     * @params: none
     * @return: none
     */
    void setAsInput();

    /**
     * Return whether or not the pin is set as an output pin
     * @params: none
     * @return: direction of pin (false = input, true = output)
     */
    bool isOutputMode();

    /**
     * Write state of pin if it is an output pin
     * @params: state to write
     * @return: -1 if the pin is an input pin
     *           0 if written successfully
     */
    int write(gpio_state state);

    /**
     * Read the state of the pin if it is an input pin
     * @params: none
     * @return: -1 if pin is an output pin
     *           0 if pin reads a LOW state
     *           1 if pin reads a HIGH state
     */
    int read();


private:
    bool isOutput; // 0 = input, 1 = output
    bool initialized; // 0 = false, 1 = true
    uint32_t pinMask;
    uint32_t portBase;
};





}
#endif /* GPIO_HPP */
