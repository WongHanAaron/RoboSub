/**
 * SysClock class acts as a delay class for the system. This delay halts the system
 * until the it has waiting the specified amount of time.
 */
#include <stdint.h>
#include "driverlib/sysctl.h"

#ifndef SYSCLOCK_HPP
#define SYSCLOCK_HPP

namespace Exodus
{

// A class that reads the current system clock to provide useful delay intervals
class SysClock {
public:
    /**
     * Initialize System Clock for delays
     * @params: none
     * @return: none
     */
    void initialize();

    /**
     * Create a delay for "counts" seconds
     * @params: number of seconds for delay (counts)
     * @return: none
     */
    void delayBy1s(int counts);

    /**
     * Create a delay for "counts" milliseconds
     * @params: number of seconds for delay (counts)
     * @return: none
     */
    void delayBy1ms(int counts);

    /**
     * Create a delay for "counts" microseconds
     * @params: number of seconds for delay (counts)
     * @return: none
     */
    void delayBy1us(int counts);

private:
    int clockFrequency;
};


}



#endif /* SYSCLOCK_HPP */
