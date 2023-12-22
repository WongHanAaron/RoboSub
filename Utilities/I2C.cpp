#include <string.h>
#include "I2C.hpp"


void Exodus::I2C::initialize(I2COptions options, tSensorCallback *pfnCallback) {
    unsigned int port;
    unsigned int module;
    unsigned int SCL;
    unsigned int SDA;
    unsigned int portBase;
    unsigned int SCLbit;
    unsigned int SDAbit;
    unsigned int IntBit;

    savedCallback = pfnCallback;
    // Load the appropriate macros based on user's choice
    switch (options) {
        case I2C0PB:
            port = SYSCTL_PERIPH_GPIOB;
            module = SYSCTL_PERIPH_I2C0;
            SCL = GPIO_PB2_I2C0SCL;
            SDA = GPIO_PB3_I2C0SDA;
            portBase = GPIO_PORTB_BASE;
            moduleBase = I2C0_BASE;
            SCLbit = GPIO_PIN_2;
            SDAbit = GPIO_PIN_3;
            IntBit = INT_I2C0;
            break;
        case I2C1PA:
            port = SYSCTL_PERIPH_GPIOA;
            module = SYSCTL_PERIPH_I2C1;
            SCL = GPIO_PA6_I2C1SCL;
            SDA = GPIO_PA7_I2C1SDA;
            portBase = GPIO_PORTA_BASE;
            moduleBase = I2C1_BASE;
            SCLbit = GPIO_PIN_6;
            SDAbit = GPIO_PIN_7;
            IntBit = INT_I2C1;
            break;
        case I2C2PE:
            port = SYSCTL_PERIPH_GPIOE;
            module = SYSCTL_PERIPH_I2C2;
            SCL = GPIO_PE4_I2C2SCL;
            SDA = GPIO_PE5_I2C2SDA;
            portBase = GPIO_PORTD_BASE;
            moduleBase = I2C2_BASE;
            SCLbit = GPIO_PIN_4;
            SDAbit = GPIO_PIN_5;
            IntBit = INT_I2C2;
            break;
        case I2C3PD:
            port = SYSCTL_PERIPH_GPIOD;
            module = SYSCTL_PERIPH_I2C3;
            SCL = GPIO_PD0_I2C3SCL;
            SDA = GPIO_PD1_I2C3SDA;
            portBase = GPIO_PORTD_BASE;
            moduleBase = I2C3_BASE;
            SCLbit = GPIO_PIN_0;
            SDAbit = GPIO_PIN_1;
            IntBit = INT_I2C3;
            break;
    }
    debugOutput = 0;
    SysCtlPeripheralEnable(port);      // Enable the alternate functionality for the GPIO
    SysCtlPeripheralEnable(module);    // Enable the peripheral

    GPIOPinConfigure(SCL);             // Configure the clock pin
    GPIOPinConfigure(SDA);             // Configure the data pin

    GPIOPinTypeI2CSCL(portBase, SCLbit);  // Configure the pin for I2C Clock functionality
    GPIOPinTypeI2C(portBase, SDAbit);     // Configure the pin for I2C functionality

    IntMasterEnable();
    I2CMInit(&I2CInst, moduleBase, IntBit, 0xff, 0xff, SysCtlClockGet());   // Initialize I2C functionality
}

void Exodus::I2C::HandleInterrupt() {
    // The HandleInterrupt function is essential to the operation of the I2C driver
    // It controls the state of the I2C structure defined by TivaWare. This function
    // triggers TivaWare's structure handling and provides functionality for
    // printing the state of the structure.

    I2CMIntHandler(&I2CInst);
    char I2CState[10];
    if (debugOutput != 0 && PrintDebug) {
        switch (I2CInst.ui8State) {
            case 0:     // Indicates STATE_IDLE
                strcpy(I2CState, "Idle");
                break;
            case 1:     // Indicates STATE_WRITE_NEXT
                strcpy(I2CState, "Write_Next");
                break;
            case 2:     // Indicates STATE_WRITE_FINAL
                strcpy(I2CState, "WriteFinal");
                break;
            case 3:     // Indicates STATE_WRITE_PAUSE
                strcpy(I2CState, "WritePause");
                break;
            case 4:     // Indicates STATE_READ_ONE
                strcpy(I2CState, "Read_One");
                break;
            case 5:     // Indicates STATE_READ_FIRST
                strcpy(I2CState, "Read_First");
                break;
            case 6:     // Indicates STATE_READ_NEXT
                strcpy(I2CState, "Read_Next");
                break;
            case 7:    // Indicates STATE_READ_FINAL
                strcpy(I2CState, "Read_Final");
                break;
            case 8:    // Indicates STATE_READ_PAUSE
                strcpy(I2CState, "Read_Pause");
                break;
            case 9:    // Indicates STATE_READ_WAIT
                strcpy(I2CState, "Read_Wait");
                break;
            case 10:    // Indicates STATE_CALLBACK
                strcpy(I2CState, "Callback");
                break;
            default:    // If the value is uncaught. It is an unknown state
                strcpy(I2CState, "Unknown");
                break;
        }
        debugOutput->printf("[I2CDebug] Interrupted with %s\n~", I2CState);
    }
}

unsigned int Exodus::I2C::write(uint_fast8_t ui8Addr, const uint8_t *pui8Data, uint_fast16_t ui16Count, bool blocking){
    // The Write function is a wrapper for TivaWare's I2CMWrite function
    // It encapsulates and provides the relevant variables for the TivaWare I2CMWrite
    // It additionally requires that a callback function be specified upon initialization
    // The callback function will flip a boolean that signifies that the I2C transaction
    // is complete. This function allows for blocking or non-blocking. Blocking will
    // wait for the I2C transaction to complete while non-blocking will proceed

    // Send a write request with the inputed parameters
    unsigned int status = I2CMWrite(&I2CInst, ui8Addr, pui8Data, ui16Count, savedCallback, &I2CInst);
    if (status == 0) {       // if the status is 0, the I2C transaction failed
        // Potentially print debug statements
        if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Failed to add write command\n~");

        return 0;
    }

    // Potentially print status
    if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Successfully added write command\n~");

    // Potential blocking
    if(blocking) {
        if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Waiting on transaction completion\n~");

        while(!TransactionComplete);
        TransactionComplete = false;
    }

    if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Read transaction complete\n~");

    return status;
}

unsigned int Exodus::I2C::read(uint_fast8_t ui8Addr, const uint8_t *pui8WriteData, uint_fast16_t ui16WriteCount,
                        uint8_t *pui8ReadData, uint_fast16_t ui16ReadCount, bool blocking) {
    // The Read function is a wrapper for TivaWare's I2CMWrite function
    // It encapsulates and provides the relevant variables for the TivaWare I2CMRead.


    // Send a read request with the inputed parameters
    unsigned int status = I2CMRead(&I2CInst, ui8Addr, pui8WriteData, ui16WriteCount,
                                   pui8ReadData, ui16ReadCount, savedCallback, &I2CInst);
    if (status == 0) {       // if the status is 0, the I2C transaction failed
        // Potentially print debug statements
        if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Failed to add read command\n~");
        return 0;
    }

    // Potentially print status
    if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Successfully added read command\n~");

    // Potential blocking
    if(blocking) {
        while(!TransactionComplete);
        TransactionComplete = false;
    }

    if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Read transaction complete\n~");

    return status;
}

void Exodus::I2C::HandleCallback(uint_fast8_t status) {
    // The HandleCallback function handles the callback from the TivaWare driver
    // TivaWare's I2C driver calls the callback function when its transaction is
    // complete. The callback contains information about the state and "success"
    // of the transaction. This function provides debug print functionality
    // This function will toggle the blocking boolean to true upon success and
    // and allow other functions to continue if it specified to be blocked

    char I2CStatus[10];
    if (debugOutput != 0 && PrintDebug) {
        switch (status) {
            case 0:     // Indicates I2CM_STATUS_SUCCESS
                strcpy(I2CStatus, "Success");
                break;
            case 1:     // Indicates I2CM_STATUS_ADDR_NACK
                strcpy(I2CStatus, "Addr_NACK");
                break;
            case 2:     // Indicates I2CM_STATUS_DATA_NACK
                strcpy(I2CStatus, "Data_NACK");
                break;
            case 3:     // Indicates I2CM_STATUS_ARB_LOST
                strcpy(I2CStatus, "Arb_NACK");
                break;
            case 4:     // Indicates I2CM_STATUS_ERROR
                strcpy(I2CStatus, "Error");
                break;
            case 5:     // Indicates I2CM_STATUS_BATCH_DONE
                strcpy(I2CStatus, "Batchdone");
                break;
            case 6:     // Indicates I2CM_STATUS_BATCH_READY
                strcpy(I2CStatus, "Batchready");
                break;
            default:    // If nothing is caught, its an invalid status
                strcpy(I2CStatus, "Unknown");
                break;
        }
        debugOutput->printf("[I2CDebug] Callback is %s\n~", I2CStatus);
    }

    if (status == 0) { // If the status is I2C_STATUS_SUCCESS
        TransactionComplete = true;
    }
    statusTest = status;
}

void Exodus::I2C::attachDebugOutput(JetsonLink* debugOutput, bool printDebug) {
    // The AttachDebugOutput function attaches a pre-initialized UART output
    // object. This I2C wrapper provides functionality to print debug statements
    // and this function chooses which UART object to print the debug statements to

    PrintDebug = printDebug;
    this -> debugOutput = debugOutput;
}

/*

    if (status == 0)        // if the status is 0, the I2C transaction failed */
bool Exodus::I2C::isDeviceAvailable(uint_fast8_t ui8Addr){
    I2CMasterSlaveAddrSet(moduleBase,ui8Addr,false);             // Initiate a write with the specified address
    I2CMasterDataPut(moduleBase, 0x1E);                          // Put null data into the buffer
    I2CMasterControl(moduleBase, I2C_MASTER_CMD_SINGLE_SEND);    // Command the I2C module to send a single command
    while(I2CMasterBusy(moduleBase));                            // Wait until MCU is done transferring.
    uint32_t status = I2CMasterErr(moduleBase);                           // Retrieve the status from the master

    switch (status) {
        case 0:         // I2C_MASTER_ERR_NONE
            if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Device Connected. Status: %d\n~", status);
            return true;
        case 0x04:      // I2C_MASTER_ERR_ADDR_ACK
            if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Address NACK error. Status: %d\n~", status);
            return false;
        case 0x08:      // I2C_MASTER_ERR_DATA_ACK
            if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Data NACK error. Status: %d\n~", status);
            return false;
        case 0x10:      // I2C_MASTER_ERR_ARB_LOST
            if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Arbitration Lost error. Status: %d\n~", status);
            return false;
        case 0x80:      // I2C_MASTER_ERR_CLK_TOUT
            if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Clock Timeout error. Status: %d\n~", status);
            return false;
        default:        // Unknown case
            if (debugOutput != 0 && PrintDebug) debugOutput->printf("[I2CDebug] Unknown status error. Status: %d\n~", status);
            return false;
    }

}
