/**
 * Class for operating Analog-Digital-Convert (ADC) pins
 * These pins will read voltages and convert them to a digital signal
 * between 0 and 4095
 */

#ifndef UTILITIES_ADC_HPP_
#define UTILITIES_ADC_HPP_

#include <stdint.h>
#include "JetsonLink.hpp"
#include "Definitions.hpp"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"

namespace Exodus {

class ADC {

public:
    /**
     * Initialize the ADC pin
     * @params: corresponding ADC pin (pinNumber)
     *          baseNumber of ADC (baseNumber)
     * @return: none
     */
    void initialize(pin pinNumber, adc_base baseNumber = BASE_0);

    /**
     * Read the value of the ADC pin
     * @params: none
     * @return: average value of the ADC pin
     */
    uint32_t read();

private:
    uint32_t adcBase;
    volatile uint32_t averageReading;
};

}

#endif /* UTILITIES_ADC_HPP_ */
