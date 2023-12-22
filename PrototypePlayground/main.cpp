/**
 * PrototypePlayground provides a "bare-bones" project for developers to
 * use as they create newer drivers
 */
#include "LED.hpp"
#include "Definitions.hpp"
#include "SysClock.hpp"
#include "JetsonLink.hpp"
using namespace Exodus;

LED leds;
SysClock Timer;
JetsonLink TiLink;

int main() {
    /* Set System Clock to 80MHz */
    SysCtlClockSet(Clk80MHz);

    /* Create a JetsonLink UART Communication */
    TiLink.initialize(UART0A,
                      115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE),
                      true);
    TiLink.printf("[MCU] Initializing PrototypePlayground\n~");

    /* Initialize Indication LEDS for 100ms Interrupt Service Routine*/
    TiLink.printf("[MCU] Initializing Indication LEDs...~");
    leds.initializeAll();
    leds.setDutyCycle(BLUE, 25);
    leds.setDutyCycle(GREEN, 20);
    TiLink.printf(" Initialized\n~");

    TiLink.printf("[MCU] System Fully Initialized\n~");


    while (true) {
// This is just a test comment to see if a merge request will catch this
// Here is another comment
	}
}

/* Externally declared in tm4c123gh6pm_startup_ccs.c */
extern "C" void TiJetsonUARTHandler(void) {
    // Loop through the UART FIFO whenever there are characters available in the UART FIFO
    /* Loop through UART FIFO when characters are available */
    while (UARTCharsAvail(TiLink.getModuleBase())) {
        TiLink.incomingByte = UARTCharGetNonBlocking(TiLink.getModuleBase()); // Store the first character in the UART FIFO
        TiLink.input[TiLink.inputIndex] = TiLink.incomingByte;                // Append the character to the input char array
        TiLink.inputIndex = (TiLink.inputIndex + 1) % 50;

        /* Look out for the EoT and signal the flag when an EoT is found, signifying for the MCU to parse the command */
        if (TiLink.incomingByte == '~') {
            TiLink.input[TiLink.inputIndex - 1] = '\0';
            if (TiLink.input[0] != '*') {
                switch (TiLink.input[0]) {
                    case SYS_RESET:
                        // Resets the system
                        SysCtlReset();
                        break;

                }
                TiLink.resetBuffer();
            } else {
                TiLink.rightShiftBuffer(1);
                TiLink.printf("[MCU] Found Command\n~");
                TiLink.foundCommand = true;
            }
        }
    }
    TiLink.resetInterrupt();
}

/* Externally declared in tm4c123gh6pm_startup_ccs.c */
extern "C" void I2C3Handler(void) {

}

/* Externally declared in tm4c123gh6pm_startup_ccs.c */
extern "C" void Timer0IntHandler(void) {

}
