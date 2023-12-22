/**
 * The LED interacts with the on-board LEDs via PWM operation.
 */
#ifndef LED_HPP_
#define LED_HPP_

#include "PWM.hpp"
#include "SystemDriver.hpp"
#include "Definitions.hpp"
#include "JetsonLink.hpp"

namespace Exodus {

class LED : public SystemDriver{

public:
    LED();
    /**
     * Initialize the specified LED PWM
     * @params: LED to initialize (option)
     * @return: successfulness of initialization
     *          0 - unsuccessful
     *          1 - successful
     */
    int initialize(led_option option, JetsonLink* tilink = 0);

    /**
     * Initialize all on-board LEDs
     * @params: none
     * @return: successfulness of initialization
     *          0 - unsuccessful
     *          1 - successful
     */
    int initializeAll();

    /**
     * Turn on/off the specified LED
     * @params: LED to toggle (option)
     * @return: none
     */
    void toggle(led_option option);

    /**
     * Turn on/off all LEDs
     * @params: none
     * @return: none
     */
    void toggleAll();

    /**
     * Set the Duty Cycle of the specified LED
     * @params: LED to brighten/dim (option)
     *          Duty Cycle to set (percentage)
     * @return: none
     */
    void setDutyCycle(led_option option, double percentage);

    /**
     * Set the Duty Cycle of all LEDs
     * @params: Duty Cycle to set (percentage)
     * @return: none
     */
    void setAllDutyCycle(double percentage);

private:
    PWMPin red;
    PWMPin green;
    PWMPin blue;

    static bool redInitialized;
    static bool blueInitialized;
    static bool greenInitialized;
};

} /* namespace Exodus */

#endif /* SYSTEM_LED_HPP_ */
