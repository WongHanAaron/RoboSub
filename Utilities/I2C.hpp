/**
 * I2C is a communication protocol for interacting with various devices using only two pins
 * The I2c bus consists of a clock pin for synchronizing communication and a data pin for
 * sending/receiving data.
 * The communication uses an address system to communicate with these devices and to
 * interact with the registers of said devices.
 */

#include <stdint.h>
#include "JetsonLink.hpp"
#include "Definitions.hpp"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "sensorlib/i2cm_drv.h"



#ifndef I2C_HPP
#define I2C_HPP

namespace Exodus {

typedef void (tSensorCallback)(void *pvData, uint_fast8_t ui8Status);


class I2C {
public:

    /**
     * Initialize I2C communication
     * @params: option for pair of I2C pins (options)
     *          callback function that is called after communication (pfnCallback)
     *
     * @return: none
     */
    void initialize(I2COptions options, tSensorCallback *pfnCallback);

    /**
     * Function called in the event of an interrupt
     * @params: none
     * @return: none
     */
    void HandleInterrupt();

    /**
     * Function called after I2C communication
     * @params: variable that returns the successfulness of the communication (status)
     * @return: none
     */
    void HandleCallback(uint_fast8_t status);

    /**
     * Function used to write to a device connected with an I2C instance
     * @params: address of device to write to (ui8Addr)
     *          data to write with first byte indicating which register to write to (pui8Data)
     *          number of bytes to write (ui16Count)
     *          boolean indicating blocking communication (blocking)
     *
     * @return: successfulness of communication
     */
    unsigned int write(uint_fast8_t ui8Addr, const uint8_t *pui8Data, uint_fast16_t ui16Count, bool blocking = true);

    /**
     * Function used to read and write to a device connected with an I2C instance
     * @params: address of device to write to (ui8Addr)
     *          data to write with first byte indicating which register to write to (pui8WriteData)
     *          number of bytes to write (ui16WriteCount)
     *          pointer to buffer of where to store read data (pui8ReadData)
     *          number of elements to read (ui16ReadCount)
     *          boolean indicating blocking communication (blocking)
     *
     * @return: successfulness of communication
     */
    unsigned int read(uint_fast8_t ui8Addr, const uint8_t *pui8WriteData, uint_fast16_t ui16WriteCount,
              uint8_t *pui8ReadData, uint_fast16_t ui16ReadCount, bool blocking = true);

    /**
     * Attach UART communication instance to print debug statements
     * @params: pointer to UART instance
     *          boolean indicating if these statements should be printed (printDebug)
     *
     * @return: none
     */
    void attachDebugOutput(JetsonLink* debugOutput, bool printDebug = true);

    /**
     * Check to see if a device is connected
     * @params: device address (ui8Addr)
     * @return: boolean indicating if a device is connected
     */
    bool isDeviceAvailable(uint_fast8_t ui8Addr);
    JetsonLink* debugOutput;
    bool PrintDebug;

private:
    unsigned int moduleBase;
    unsigned int statusTest;

    void PingDevice(uint_fast8_t ui8Addr);

    tI2CMInstance I2CInst;
    tSensorCallback* savedCallback;
    bool TransactionComplete;


};

}


#endif /* I2C_HPP */
