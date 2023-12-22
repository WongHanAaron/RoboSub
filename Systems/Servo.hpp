/**
 * The Servo class interacts with a servo motor over PWM operation. The
 * PWM is used for determining how far to rotate the motor.
 */
#include <stdint.h>
#include "PWM.hpp"
#include "Definitions.hpp"

#ifndef SYSTEM_SERVO_HPP_
#define SYSTEM_SERVO_HPP_

namespace Exodus {

class Servo {
public:
    /**
     * Initialize the servo motor
     * @params: pin to use for rotating the motor (option
     * @return: none
     */
    void initialize(PWMPinOptions option);

    /**
     * Rotate the motor between 0 and 180 degrees
     * @params: number of degrees to rotate (degrees)
     * @return: none
     */
    void rotate(uint32_t degrees);

    /**
     * Set the period of the servo motor
     * @params: period to set the servoPin (period)
     * @return: none
     */
    void setPeriod(uint32_t period);

    /**
     * Get the width of the servo motor
     * @params: none
     * @return: width of servoPin
     */
    uint32_t getWidth();

    /**
     * Set the width of the servoPin
     * @params: width to set (width)
     * @return: none
     */
    void setWidth(uint32_t width);

private:
    PWMPin servoPin;
    uint32_t position;
};

} /* namespace Exodus */

#endif /* SYSTEM_SERVO_HPP_ */
