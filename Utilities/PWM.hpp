/**
 * PWMPin class interacts with Pulse-Width-Modulation(PWM) pins of TM4C123 micro-controller.
 * These pins turn on/off for a certain period of specified time and results in the production
 * of non-binary voltages (namely voltages ranging between 0 and 3.3V).
 */
#include "Definitions.hpp"
#include <stdint.h>
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"



#ifndef PWM_HPP
#define PWM_HPP

namespace Exodus {

// The TI has 2 modules, each module has 4 generators. Each generator can only have one period and counting configuration
// Various pins might have to share generator period and configuration.

// In the PWMPinOptions, the first 3 char represents which pin the PWM is output from, the following 2 represents which module
// the generators are from, the last 2 represents which generator generates the period and counting

class PWMPin {
public:
    PWMPin();
    /**
     * Initialize PWMPin for operation
     * @params: desired pin to set for PWM operation (option)
     *          conversion ratio for making it easier to relate desired width to the pin's frequency (conversionRatio)
     *          desired period for PWM (period)
     *          configuration settings for PWM operation (config)
     *          clock settings for PWM (clock)
     *
     * @return: none
     */
    void initialize(PWMPinOptions option, float conversionRatio = 1.25, uint32_t width = 1, uint32_t period = 2000, uint32_t config = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC, uint32_t clock = SYSCTL_PWMDIV_64);
    /**
     * Directly set the period for the PWM
     * @params: desired period to set (period)
     * @return: none
     */
    void setPeriod(uint32_t period);

    /**
     * Set period relative to the conversion ratio
     * Period to Set = period * conversionRatio
     * @params: desired period to set (period)
     *          desired conversion ratio (conversionRatio)
     *
     * @return: none
     */
    void setPeriod(uint32_t period, float conversionRatio);

    /**
     * Set the width of the pin based on the desired duty cycle to set
     * Width = period * percentage/100
     * @params: desired duty cycle (percentage)
     * @return: none
     */
    void setDutyCycle(double percentage);

    /**
     * Set the width of the pin
     * @params: desired width (width)
     * @return: none
     */
    void setWidth(uint32_t width);

    /**
     * Set the width of the pin relative to the conversion ratio
     * Width to Set = width * conversionRatio
     * @params: desired width to set (width)
     *          desired conversion ratio (conversionRatio)
     *
     * @return: none
     */
    void setWidth(uint32_t width, float conversionRatio);

    /**
     * Enable PWMPin for operation
     * @params: boolean to determine the setting for the PWMPin (enabled)
     * @return: none
     */
    void setOutput(bool enabled);

    /**
     * Get the current width of the PWMPin
     * @params: none
     * @return: width of the PWMPin
     */
    uint32_t getWidth();

    /**
     * Get the current period of the PWMPin
     * @params: none
     * @return: period of the PWMPin
     */
    uint32_t getPeriod();

    float ConversionRatio;

private:
    uint32_t pwmBase;
    uint32_t pwmGen;
    uint32_t pwmOut;
    uint32_t pwmOutBit;
    uint32_t width;
};

}


#endif /* RbpiLink_HPP */
