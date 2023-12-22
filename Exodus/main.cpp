/**
 * NOTE: Toggle "runningMiniBotMode" dependent on program operation
 * Program Brief:
 * Main Loop:
 *  1. Initialize all system devices/sensors/motors and PIDs
 *  2. Read values from system sensors
 *  3. Calculate PID values to adjust motors for the mission state
 *  4. Update the Mission Computer
 *
 * Helper Functions:
 *  1. I2C Callback function
 *  2. Real Time Interrupt Initialization (100ms)
 *  3. PID disable function
 *  4. Print Menu function to inform mission control of valid commands
 *  5. UART Input Reading function for parsing/handling commands
 *
 * External Interrupt Handler Functions:
 *  1. UART Handler for processing Mission Control Commands
 *  2. I2C interrupt handler
 *  3. 100ms Interrupt for setting trigger flags
 *
 * NOTES:
 *   - The system uses 100ms interrupt to set the trigger flags for the main loop
 *   - Uses an I2C interrupt for debugging purposes
 *   - Uses a UART Interrupt to process and execute Mission Control Commands
 */
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "LED.hpp"
#include "Definitions.hpp"
#include "SysClock.hpp"
#include "JetsonLink.hpp"
#include "PWM.hpp"
#include "MiniBot.hpp"
#include "Motor.hpp"
#include "ADC.hpp"
#include "GPIO.hpp"
#include "BNO055.hpp"
#include "MS5837.hpp"
#include "I2C.hpp"
#include "PID.hpp"
#include "Servo.hpp"
using namespace Exodus;

bool runningMiniBotMode = false;

struct Motors {
    Motor subMotors;
    MiniBot car;
    int motorPWM[4];
    float verticalPWM;
    float rotationPWM;
    bool forwardTrigger;
    bool rotationTrigger;
    float forward_destination;
    float rotation_destination;
} motors;

struct Camera {
    Servo cameraServo;
    PID pixelPID;
    float camCenterX;
    float camCenterY;
    float pixelSample;
    uint32_t pixelSampleCount;

} camera;

struct ADC_Peripheral {
    ADC peripheral;
    bool readTrigger;
    float measurement;
};

template <typename T>
struct Sensor {
    T peripheral;
    float measurement;
};

LED leds;
SysClock Timer;
JetsonLink TiLink;
PWMPin Light;
ADC_Peripheral battery;
ADC_Peripheral fakeDepthSensor;
Sensor<BNO055> imu;
Sensor<MS5837> depthSensor;
I2C I2C3;
PID headingPID;
PID pitchPID;
PID depthPID;

/**
 * Initialize the Exodus System
 */
void initializeSystem();

/**
 * Callback function for I2C communication
 */
void I2CCallback(void *pvCallbackData, uint_fast8_t ui8Status);

/**
 * Initialize 100ms interrupt
 */
void InitRTI();

/**
 * Stop all PIDs by toggling the 'IsRunning' boolean to false
 */
void stopPIDs();

/**
 * Send possible list of commands to Mission Control
 */
void printHelpMenu();

/**
 * Read UART Input Command
 */
void readInput(char *input);

/**
 * Check to See if PIDs are Triggered and Running
 */
bool arePIDsTriggeredAndRunning();

/**
 * Make PWM value between 1200 and 1800
 */
void normalizePWM(int& pwmSignal);

int main() {

    /* *
     * Initialize all system peripherals:
     * - On-board LEDs
     * - Sub/MiniBot motors
     * - Delay Timer
     * - Battery ADC and fakeDepthSensor ADC
     * - I2C Peripherals (IMU and Depth Sensor)
     * - PIDs (headingPID, pitchPID, depthPID and camera.pixelPID)
     * - Real Time Interrupt
     */
    initializeSystem();

    while (true) {
        /* Check for command from Mission Control */
        while(TiLink.foundCommand) {
            readInput(TiLink.input);
        }

        /* Get IMU Readings */
        if (imu.peripheral.isTriggered()) {
            if (imu.peripheral.isInitialized()) {
                imu.peripheral.read();
            }
            imu.peripheral.setTriggerFlag(false);
        }

        /* Get Depth Sensor Reading */
        if (depthSensor.peripheral.isTriggered()) {
            if (depthSensor.peripheral.isInitialized()) {
                depthSensor.peripheral.read();
            }
            if (motors.car.isRunning()) {
                fakeDepthSensor.measurement = fakeDepthSensor.peripheral.read()*500.0f/4096.0f + 900.0f;
            }
            depthSensor.peripheral.setTriggerFlag(false);
        }

        /* Get Battery Reading */
        if (battery.readTrigger) {
            battery.measurement = ((float)battery.peripheral.read()) * 3.3f / 4096.0f;
            battery.readTrigger = false;
        }

        /* Check if a PID is running and if the PIDs are triggered */
        if (arePIDsTriggeredAndRunning()) {
            motors.motorPWM[0] = SUB_MOTOR_ZERO;
            motors.motorPWM[1] = SUB_MOTOR_ZERO;
            motors.motorPWM[2] = SUB_MOTOR_ZERO;
            motors.motorPWM[3] = SUB_MOTOR_ZERO;

            if (depthPID.isTriggered()) {
                if (depthPID.isRunning()) {
                    depthPID.Compute(&motors.verticalPWM, depthSensor.peripheral.pressure());
                    motors.motorPWM[0] += motors.verticalPWM;
                    motors.motorPWM[1] += motors.verticalPWM;
                }
                depthPID.setTriggerFlag(false);
            }

            if (headingPID.isTriggered()) {
                if(headingPID.isRunning() and !camera.pixelPID.isRunning()) {
                    headingPID.Compute(&motors.rotationPWM, imu.peripheral.eul[0]);
                    motors.motorPWM[2] -= motors.rotationPWM;
                    motors.motorPWM[3] += motors.rotationPWM;

                }
                headingPID.setTriggerFlag(false);
            }

            if (motors.forwardTrigger) {
                if (fabs(motors.forward_destination) > 1.0f) {
                    motors.motorPWM[2] += motors.forward_destination;
                    motors.motorPWM[3] += motors.forward_destination;
                }
                motors.forwardTrigger = false;
            }

            if (motors.rotationTrigger) {
                if (fabs(motors.rotation_destination) > 1.0f) {
                    motors.motorPWM[2] -= motors.rotation_destination;
                    motors.motorPWM[3] += motors.rotation_destination;
                }
                motors.rotationTrigger = false;
            }

            if (camera.pixelPID.isTriggered()) {
                if (!headingPID.isRunning() and camera.pixelPID.isRunning()) {
                    camera.pixelSampleCount++;
                    if (camera.pixelSampleCount > camera.pixelSample) {
                        camera.pixelSampleCount = 0;
                        camera.pixelPID.Compute(&motors.rotationPWM, camera.camCenterX);
                        motors.motorPWM[2] -= motors.rotationPWM;
                        motors.motorPWM[3] += motors.rotationPWM;
                    }
                }
                camera.pixelPID.setTriggerFlag(false);
            }

            /* Make sure that the motors are within the PWM Range */
            normalizePWM(motors.motorPWM[0]);
            normalizePWM(motors.motorPWM[1]);
            normalizePWM(motors.motorPWM[2]);
            normalizePWM(motors.motorPWM[3]);

            if (motors.car.isRunning()) {
                motors.car.setLeftWidth((unsigned int) motors.motorPWM[2]);
                motors.car.setRightWidth((unsigned int) motors.motorPWM[3]);
            } else {
                motors.subMotors.setLeftVerticalWidth((unsigned int) motors.motorPWM[0]);
                motors.subMotors.setRightVerticalWidth((unsigned int) motors.motorPWM[1]);
                motors.subMotors.setLeftHorizontalWidth((unsigned int) motors.motorPWM[2]);
                motors.subMotors.setRightHorizontalWidth((unsigned int) motors.motorPWM[3]);
            }
        }

        /* If Triggered, Update Mission Control */
        if (TiLink.isTriggered()) {
            TiLink.sendFloat(CUR_FORWARD, motors.forward_destination);
            TiLink.sendFloat(ROTATION_PWM, motors.rotationPWM);
            TiLink.sendFloat(VERTICAL_PWM, motors.verticalPWM);
            TiLink.sendFloat(PITCH_PWM, 0.0);
            TiLink.sendFloat(CUR_HEADING, fmod(360.0f - imu.peripheral.eul[0], 360.0f));
            TiLink.sendFloat(CUR_PITCH, imu.peripheral.eul[2]);
            TiLink.sendFloat(CUR_BATTERY, battery.measurement);
            if (motors.car.isRunning()) {
                TiLink.sendFloat(MOTOR3_PWM, motors.car.getLeftWidth());
                TiLink.sendFloat(MOTOR4_PWM, motors.car.getRightWidth());
                TiLink.sendFloat(CUR_DEPTH, fakeDepthSensor.measurement);
            } else {
                TiLink.sendFloat(MOTOR1_PWM, motors.subMotors.getLeftVerticalWidth());
                TiLink.sendFloat(MOTOR2_PWM, motors.subMotors.getRightVerticalWidth());
                TiLink.sendFloat(MOTOR3_PWM, motors.subMotors.getLeftHorizontalWidth());
                TiLink.sendFloat(MOTOR4_PWM, motors.subMotors.getRightHorizontalWidth());
                TiLink.sendFloat(CUR_DEPTH, depthSensor.peripheral.pressure());
                TiLink.sendFloat(SERVO, camera.cameraServo.getWidth());
                TiLink.sendFloat(BIG_LIGHT, Light.getWidth());
            }
            TiLink.sendFloat(CUR_ACCEL_X, imu.peripheral.accel[0]);
            TiLink.sendFloat(CUR_ACCEL_Y, imu.peripheral.accel[1]);
            TiLink.sendFloat(CUR_ACCEL_Z, imu.peripheral.accel[2]);
            TiLink.sendFloat(CAM_CENTER_REQ, 0);

            TiLink.setTriggerFlag(false);
        }
    }
}

void initializeSystem() {
    int headingDeadband, depthDeadband, pixelDeadband; // deadbands indicate the band at which change will not occur
                                                           // For instance, the deadband of sub motors are 30 due to 30 being
                                                           // the range about the 1500 PWM at which the motors will not rotate
    /* Set System Clock to 80MHz */
    SysCtlClockSet(Clk80MHz);

    /* Create a JetsonLink UART Communication */
    TiLink.initialize(UART0A,
                      115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE),
                      true);
    TiLink.printf("[MCU] Initializing Exodus\n~");

    /* Initialize Indication LEDS for 100ms Interrupt Service Routine*/
    TiLink.printf("[MCU] Initializing Indication LEDs...~");
    leds.initializeAll();
    leds.setDutyCycle(RED, 25);
    leds.setDutyCycle((runningMiniBotMode) ? BLUE : GREEN, 20); // If running minibot, turn on BLUE LED, else turn on GREEN LED
    TiLink.printf(" Initialized\n~");

    /* Initialize Camera Servo and Light*/
    // CameraServo.initialize(PA6M1G1); // Disabled servo pin
    Light.initialize(PA7M1G1);

    /* Initialize system motors */
    TiLink.printf("[MCU] Initializing Motors...~");
    motors.car.setDeviceRunning(runningMiniBotMode);
    if (motors.car.isRunning()) {
        TiLink.printf("Running Mini-bot Mode...~");
        motors.car.initialize(&TiLink, PC4M0G3, PC5M0G3, PE5M0G2, PE4M0G2);
        headingDeadband = 0;
        depthDeadband = 0;
        pixelDeadband = 0;
    } else {
        motors.subMotors.initialize(PE5M0G2, PC4M0G3, PC5M0G3, PA6M1G1);
        TiLink.printf("Running Submarine Mode...~");
        Light.setWidth(1100);
        headingDeadband = SUB_MOTOR_DEADBAND_HALF;
        depthDeadband = SUB_MOTOR_DEADBAND_HALF;
        pixelDeadband = SUB_MOTOR_DEADBAND_HALF;
    }
    TiLink.printf(" Initialized\n~");

    /* Initialize Delay Timer */
    TiLink.printf("[MCU] Initializing Timer...~");
    Timer.initialize();
    TiLink.printf(" Initialized\n~");

    /* Initialize Analog-Digital Converters */
    TiLink.printf("[MCU] Initializing Analog Digital Converters...~");
    battery.peripheral.initialize(PB4);
    if (motors.car.isRunning()){
        // If we are running the mini-bot, we want to simulate our depth sensor using a potentiometer
        fakeDepthSensor.peripheral.initialize(PD2, BASE_1);
    }
    TiLink.printf(" Initialized\n~");

    /*Initialize Temporary GPIO pin to test if the I2C circuit is attached*/
    TiLink.printf("[MCU] Initializing GPIO for I2C3 Test...~");
    GPIO I2CTester;
    I2CTester.initialize(PB6, input);          // Initialize the pin to input mode on PB6 (I2C3's Clock Pin)
    TiLink.printf(" Initialized\n~");
    Timer.delayBy1ms(10);

    if (motors.car.isRunning() or I2CTester.read() == high) {
        TiLink.printf("[MCU] I2C Circuit is Attached. Proceeding with I2C and Sensor Initialization\n~");
        /* Initialize I2C module */
        TiLink.printf("[MCU] Initializing I2C...~");
        I2C3.initialize(I2C3PD, I2CCallback);               // Initializes PB6 & PD0 to be SCL, PB7 & PD1 to be SDA
        I2C3.attachDebugOutput(&TiLink, false);
        TiLink.printf(" Initialized\n~");

        /* Initialize Depth Sensor */
        TiLink.printf("[MCU] Initializing Depth Sensor... ~");
        if(I2C3.isDeviceAvailable(0x76)){
            depthSensor.peripheral.initialize(&I2C3);
            depthSensor.peripheral.setModel(0);          //MS5837_30BA
            TiLink.printf(" Initialized\n~");
        } else {
            TiLink.printf(" FAILED\n~");
        }

        /* Initialize IMU Sensor */
        TiLink.printf("[MCU] Initializing IMU Sensor...~");
        if (I2C3.isDeviceAvailable(0x28)) {
            imu.peripheral.initialize(&I2C3);
            imu.peripheral.setXAxisOffset(16.2 * 1e-6);
            imu.peripheral.setYAxisOffset(-27.3 * 1e-6);
            TiLink.printf(" Initialized\n~");
        } else {
            TiLink.printf(" FAILED\n~");
        }
    } else {
        TiLink.printf("[MCU] I2C Circuit is NOT-attached. Skipping I2C and Sensor Initialization\n~");
    }

    /* Initialize Depth PID */
    TiLink.printf("[MCU] Initializing Depth PID...~");
    depthPID.Initialize(7, 0.1, 2, Decimal, &TiLink, false);
    depthPID.SetOutputLimits(-300, 300);
    depthPID.SetDeadband(0, -depthDeadband, depthDeadband);
    depthPID.setDeviceRunning(false);
    TiLink.printf(" Initialized\n~");

    /* Initialize Heading PID */
    TiLink.printf("[MCU] Initializing Heading PID...~");
    headingPID.Initialize(2, 0, 7, Angle, &TiLink, false);
    headingPID.SetOutputLimits(-300, 300);
    headingPID.SetDeadband(0, -headingDeadband, headingDeadband);
    headingPID.setDeviceRunning(false);
    TiLink.printf(" Initialized\n~");

    /* Initialize Pixel PID */
    TiLink.printf("[MCU] Initializing Pixel-based PID...~");
    camera.pixelPID.Initialize(1, 0, 0, Decimal);
    camera.pixelPID.SetOutputLimits(-300, 300);
    camera.pixelPID.SetDeadband(0, -pixelDeadband, pixelDeadband);
    camera.pixelPID.setDeviceRunning(false);
    TiLink.printf(" Initialized\n~");

    /* Initialize Pitch PID */
    TiLink.printf("[MCU] Initializing Pitch-based PID...~");
    pitchPID.Initialize(0, 0, 0, Angle);
    pitchPID.SetOutputLimits(-300, 300);
    pitchPID.SetDeadband(0, -10, 10);
    pitchPID.setDeviceRunning(false);
    TiLink.printf(" Initialized\n~");

    /* Initialize the Real Time Interrupt Service a.k.a the 100ms Interrupt */
    TiLink.printf("[MCU] Initializing Real Time Interrupt...~");
    InitRTI();
    TiLink.printf(" Initialized\n~");
    TiLink.printf("[MCU] System Fully Initialized\n~");
}

void I2CCallback(void *pvCallbackData, uint_fast8_t ui8Status) {
    I2C3.HandleCallback(ui8Status);         // Tells the I2C object to execute the next step in the state-machine
}

void InitRTI() {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);           // Enables the Timer 0 Peripheral
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);        // Sets the timer mode to periodic

    // TimerLoadSet(TIMER0_BASE, TIMER_A, RTIPeriod);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()/10) - 1);      // Sets the timer to expire at approximately 100ms

    IntEnable(INT_TIMER0A);                                 // Enable the interrupts for Timer 0
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);        // Sets the interrupts to trigger when Timer 0 experiences a timeout
    IntMasterEnable();                                      // Enable all interrupts

    TimerEnable(TIMER0_BASE, TIMER_A);                      // Enable the interrupt
}

void stopPIDs() {
    depthPID.setDeviceRunning(false);
    headingPID.setDeviceRunning(false);
    camera.pixelPID.setDeviceRunning(false);
    pitchPID.setDeviceRunning(false);
}

void printHelpMenu() {
    TiLink.printf("[MCU Help Menu]\n");
    TiLink.printf("   command     description\n~");
    TiLink.printf("   depthtest   runs a dedicated function loop to constantly reads and prints the depth sensor\n~");
    TiLink.printf("   imutest     runs a dedicated function loop to constantly reads and prints the imu sensor\n~");
    TiLink.printf("   imucal      runs the imu calibration function\n~");
    TiLink.printf("   i2cdebug    toggles the debug mode for the I2C driver\n~");
    TiLink.printf("   tDepthPID   toggles the depth-based PID\n~");
    TiLink.printf("   tHeadingPID toggles the heading-based PID\n~");
    TiLink.printf("   tcamera.pixelPID   toggles the pixel-based PID\n~");
    TiLink.printf("   tPitchPID   toggles the pitch-based PID\n~");
    TiLink.printf("\n~");
}

void readInput(char *input) {
    TiLink.printf("[MCU] Received \"%s\"\n~", TiLink.input);

    if (strcmp(input, "help") == 0) {
       printHelpMenu();
    } else if (strcmp(input, "depthtest") == 0) {
       depthSensor.peripheral.test();
    } else if (strcmp(input, "imutest") == 0) {
       imu.peripheral.test();
    } else if (strcmp(input, "i2cdebug") == 0) {
       // A toggle for enabling and disabling the I2C debug prints
       if (I2C3.PrintDebug) {
          TiLink.printf("[MCU] Toggling I2C Debug off\n~");
          I2C3.PrintDebug = false;
       } else {
          TiLink.printf("[MCU] Toggling I2C Debug on\n~");
          I2C3.PrintDebug = true;
       }
    } else if (strcmp(input, "tDepthPID") == 0) {
        // A toggle for enabling and disabling the depth PID
       if (depthPID.isRunning()) {
          TiLink.printf("[MCU] Toggling Depth PID off\n~");
          depthPID.setDeviceRunning(false);
       } else {
          TiLink.printf("[MCU] Toggling Depth PID on\n~");
          depthPID.setDeviceRunning(true);
       }
    } else if (strcmp(input, "tHeadingPID") == 0) {
        // A toggle for enabling and disabling the heading PID
       if (headingPID.isRunning()) {
          TiLink.printf("[MCU] Toggling Heading PID off\n~");
          headingPID.setDeviceRunning(false);
       } else {
          TiLink.printf("[MCU] Toggling Heading PID on\n~");
          headingPID.setDeviceRunning(true);
          // if we turn Heading PID on, we need to turn off Pixel PID
          TiLink.printf("[MCU] Toggling Pixel PID off\n~");
          camera.pixelPID.setDeviceRunning(false);
       }
    } else if (strcmp(input, "tcamera.pixelPID") == 0) {
        // A toggle for enabling and disabling the pixel PID
       if (camera.pixelPID.isRunning()) {
          TiLink.printf("[MCU] Toggling Pixel PID off\n~");
          camera.pixelPID.setDeviceRunning(false);
       } else {
          TiLink.printf("[MCU] Toggling Pixel PID on\n~");
          camera.pixelPID.setDeviceRunning(true);
          // if we turn Pixel PID on, we need to turn off Pixel PID
          TiLink.printf("[MCU] Toggling Heading PID off\n~");
          headingPID.setDeviceRunning(false);
       }
    } else if (strcmp(input, "tPitchPID") == 0) {
        // A toggle for enabling and disabling the pitch PID
       if (pitchPID.isRunning()){
          TiLink.printf("[MCU] Toggling Pitch PID off\n~");
          pitchPID.setDeviceRunning(false);
       } else {
          TiLink.printf("[MCU] Toggling Pitch PID on\n~");
          pitchPID.setDeviceRunning(true);
       }
    } else if (strcmp(input, "imucal") == 0) {
       imu.peripheral.calibrate();
       TiLink.sendFloat(CAL_MAG_X, imu.peripheral.getXAxisOffset());
       TiLink.sendFloat(CAL_MAG_Y, imu.peripheral.getYAxisOffset());
    } else {
       TiLink.printf("[MCU] Unrecognized command. Type \"help\" for all possible commands\n~");
    }
    TiLink.resetBuffer();
}

bool arePIDsTriggeredAndRunning() {
    return (depthPID.isRunning() or headingPID.isRunning() or camera.pixelPID.isRunning() or fabs(motors.forward_destination > 1.0f))
            and (depthPID.isTriggered() or headingPID.isTriggered() or motors.forwardTrigger or motors.rotationTrigger or camera.pixelPID.isTriggered());
}

void normalizePWM(int& pwmSignal) {
    if (pwmSignal < 1200) {
        pwmSignal = 1200;
    } else if (pwmSignal > 1800) {
        pwmSignal = 1800;
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
                    case DES_HEADING:
                        // Updates the desired heading value to be achieved
                        headingPID.setSetPoint(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] Desired Heading Updated to: %0.3f\n~", headingPID.getSetPoint());
                        break;
                    case DES_DEPTH:
                        // Updates the desired depth value to be achieved
                        depthPID.setSetPoint(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] Desired Depth Updated to: %0.3f\n~", depthPID.getSetPoint());
                        break;
                    case DES_FORWARD:
                        // Updates the desired forward PWM thrust
                        motors.forward_destination = charToFloat(TiLink.input + 1);
                        TiLink.printf("[MCU] Desired Forward Updated to: %0.3f\n~", motors.forward_destination);
                        break;
                    case DES_ROTATION:
                        // Updates the desired rotation PWM thrust
                        motors.rotation_destination = charToFloat(TiLink.input + 1);
                        TiLink.printf("[MCU] Desired Rotation Updated to: %0.3f\n~", motors.rotation_destination);
                        break;
                    case DES_PITCH:
                        // Update the Pitch PID setPoint
                        pitchPID.setSetPoint(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] Desired Pitch Updated to: %0.3f\n~", pitchPID.getSetPoint());
                        break;
                    case DEPTH_P:
                        // updates the Proportional component of the depth PID
                        depthPID.resetPID();
                        depthPID.setKp(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] P-Gain for Depth Updated to: %0.3f\n~", depthPID.getKp());
                        break;
                    case DEPTH_I:
                        // updates the Integral component of the depth PID
                        depthPID.resetPID();
                        depthPID.setKi(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] I-Gain for Depth Updated to: %0.3f\n~", depthPID.getKi());
                        break;
                    case DEPTH_D:
                        // updates the Derivative component of the depth PID
                        depthPID.resetPID();
                        depthPID.setKd(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] D-Gain for Depth Updated to: %0.3f\n~", depthPID.getKd());
                        break;
                    case HEADING_P:
                        // updates the Proportional component of the heading PID
                        headingPID.resetPID();
                        headingPID.setKp(charToFloat(TiLink.input+1));
                        TiLink.printf("[MCU] P-Gain for Heading Updated to: %0.3f\n~", headingPID.getKp());
                        break;
                    case HEADING_I:
                        // updates the Integral component of the heading PID
                        headingPID.resetPID();
                        headingPID.setKi(charToFloat(TiLink.input+1));
                        TiLink.printf("[MCU] I-Gain for Heading Updated to: %0.3f\n~", headingPID.getKi());
                        break;
                    case HEADING_D:
                        // updates the Derivative component of the heading PID
                        headingPID.resetPID();
                        headingPID.setKd(charToFloat(TiLink.input+1));
                        TiLink.printf("[MCU] D-Gain for Heading Updated to: %0.3f\n~", headingPID.getKd());
                        break;
                    case PIXEL_P:
                        // updates the Proportional component of the pixel PID
                        camera.pixelPID.resetPID();
                        camera.pixelPID.setKp(charToFloat(TiLink.input+1));
                        TiLink.printf("[MCU] P-Gain for Pixel Updated to: %0.3f x 10^-3\n~", camera.pixelPID.getKp()*1000);
                        break;
                    case PIXEL_I:
                        // updates the Integral component of the pixel PID
                        camera.pixelPID.resetPID();
                        camera.pixelPID.setKi(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] I-Gain for Pixel Updated to: %0.3f x 10^-3\n~", camera.pixelPID.getKi()*1000);
                        break;
                    case PIXEL_D:
                        // updates the Derivative component of the pixel PID
                        camera.pixelPID.resetPID();
                        camera.pixelPID.setKd(charToFloat(TiLink.input + 1));
                        TiLink.printf("[MCU] D-Gain for Pixel Updated to: %0.3f x 10^-3\n~", camera.pixelPID.getKd()*1000);
                        break;
                    case PITCH_P:
                        // updates the Proportional component of the pitch PID
                        pitchPID.resetPID();
                        pitchPID.setKp(charToFloat(TiLink.input+1));
                        TiLink.printf("[MCU] P-Gain for Pitch Updated to: %0.3f\n~", pitchPID.getKp());
                        break;
                    case PITCH_I:
                        // updates the Integral component of the pitch PID
                        pitchPID.resetPID();
                        pitchPID.setKi(charToFloat(TiLink.input+1));
                        TiLink.printf("[MCU] I-Gain for Pitch Updated to: %0.3f\n~", pitchPID.getKi());
                        break;
                    case PITCH_D:
                        // updates the Derivative component of the pitch PID
                        pitchPID.resetPID();
                        pitchPID.setKd(charToFloat(TiLink.input+1));
                        TiLink.printf("[MCU] D-Gain for Pitch Updated to: %0.3f\n~", pitchPID.getKd());
                        break;
                    case MOTOR1_PWM:
                        // If sub motors are running, update Motor 1 PWM
                        if (!motors.car.isRunning()) {
                            motors.subMotors.setLeftVerticalWidth((unsigned int)charToFloat(TiLink.input+1));
                            stopPIDs();
                        }
                        break;
                    case MOTOR2_PWM:
                        // If sub motors are running, update Motor 1 PWM
                        if (!motors.car.isRunning()) {
                            motors.subMotors.setRightVerticalWidth((unsigned int)charToFloat(TiLink.input+1));
                            stopPIDs();
                        }
                        break;
                    case MOTOR3_PWM:
                        // Update Motor 3 PWM
                        if (motors.car.isRunning()) {
                            motors.car.setLeftWidth((unsigned int)charToFloat(TiLink.input+1));
                        } else {
                            motors.subMotors.setLeftHorizontalWidth((unsigned int)charToFloat(TiLink.input+1));
                        }
                        stopPIDs();
                        break;
                    case MOTOR4_PWM:
                        // Update Motor 4 PWM
                        if (motors.car.isRunning()) {
                            motors.car.setRightWidth((unsigned int)charToFloat(TiLink.input+1));
                        } else {
                            motors.subMotors.setRightHorizontalWidth((unsigned int)charToFloat(TiLink.input+1));
                        }
                        stopPIDs();
                        break;
                    case PID_MODE:
                        break;
                    case DEBUG_MODE:
                        break;
                    case PIXEL_SAMPLE:
                        // Update the pixel PID sample rate
                        camera.pixelSample = (int)charToFloat(TiLink.input + 1);
                        TiLink.printf("[MCU] Received a pixel PID sample rate: %0.3f\n~", camera.pixelSample);
                        break;
                    case CAM_CENTER_X:
                        // Update Camera Center on the X-Axis
                        camera.camCenterX = charToFloat(TiLink.input + 1);
                        // TiLink.printf("[MCU] Received a target center x value: %0.3f\n~", cam_center_x);
                        break;
                    case CAM_CENTER_Y:
                        // Update Camera Center on the Y-Axis
                        camera.camCenterY = charToFloat(TiLink.input + 1);
                        // TiLink.printf("[MCU] Received a target center y value: %0.3f\n~", cam_center_y);
                        break;
                    case CAL_MAG_X:
                        imu.peripheral.setXAxisOffset(charToFloat(TiLink.input + 1) * 1e-6);
                        TiLink.printf("[MCU] Received an x-axis mag calibration value: %0.3f\n~", imu.peripheral.getXAxisOffset() * 1e6);
                        break;
                    case CAL_MAG_Y:
                        imu.peripheral.setYAxisOffset(charToFloat(TiLink.input + 1) * 1e-6);
                        TiLink.printf("[MCU] Received an y-axis mag calibration value %0.3f\n~", imu.peripheral.getYAxisOffset() * 1e6);
                        break;
                    case SET_DEPTH_PID:
                        // Setting the state of the Depth PID
                        if (charToFloat(TiLink.input + 1) <= 500) {
                            TiLink.printf("[MCU] Setting Depth PID off\n~");
                            depthPID.setDeviceRunning(false);
                            motors.subMotors.setAllWidths(1500);
                        } else {
                            TiLink.printf("[MCU] Setting Depth PID on\n~");
                            depthPID.setDeviceRunning(true);
                            depthPID.setSetPoint(depthSensor.peripheral.pressure());
                            TiLink.printf("[MCU] Desired Depth Updated to: %0.3f\n~", depthSensor.peripheral.pressure());
                        }
                        break;
                    case SET_HEADING_PID:
                        // Setting the state of the Heading PID
                        if (charToFloat(TiLink.input + 1) <= 500) {
                            TiLink.printf("[MCU] Setting Heading PID off\n~");
                            headingPID.setDeviceRunning(false);
                            motors.subMotors.setAllWidths(1500);
                        } else {
                            TiLink.printf("[MCU] Setting Heading PID on\n~");
                            headingPID.setDeviceRunning(true);
                            headingPID.setSetPoint(imu.peripheral.eul[0]);
                            TiLink.printf("[MCU] Desired Heading Updated to: %0.3f\n~", imu.peripheral.eul[0]);
                        }
                        break;
                    case SET_PIXEL_PID:
                        // Setting the state of the Pixel PID
                        if (charToFloat(TiLink.input + 1) <= 500) {
                            TiLink.printf("[MCU] Setting Pixel PID off\n~");
                            camera.pixelPID.setDeviceRunning(false);
                            motors.subMotors.setAllWidths(1500);
                        } else {
                            TiLink.printf("[MCU] Setting Pixel PID on\n~");
                            camera.pixelPID.setDeviceRunning(true);
                            camera.camCenterX = 0.0;
                        }
                        break;
                    case SET_PITCH_PID:
                        // Set the state of the Pitch PID
                        if (charToFloat(TiLink.input + 1) <= 500) {
                            TiLink.printf("[MCU] Setting Pitch PID off\n~");
                            pitchPID.setDeviceRunning(false);
                            motors.subMotors.setAllWidths(1500);
                        } else {
                            TiLink.printf("[MCU] Setting Pitch PID on\n~");
                            pitchPID.setDeviceRunning(true);
                            pitchPID.setSetPoint(imu.peripheral.eul[2]);
                        }
                        break;
                    case SERVO:
                        // Set the Rotation of the servo
                        unsigned int servoPWM = (unsigned int)charToFloat(TiLink.input+1);
                        // saturate pwm values going into the server if too high or low
                        if (servoPWM < 500) {
                            servoPWM = 500;
                        } else if (servoPWM > 1600) {
                            servoPWM = 1600;
                        }
                        camera.cameraServo.setWidth(servoPWM);
                        break;
                    case BIG_LIGHT:
                        // Set the brightness of the Light by adjusting the PWM Width
                        unsigned int lightPWM = (unsigned int)charToFloat(TiLink.input+1);
                        // saturate values if too high or too low
                        if (lightPWM < 1100){
                            lightPWM = 1100;
                        } else if (lightPWM > 1900){
                            lightPWM = 1900;
                        }
                        Light.setWidth(lightPWM);
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
    I2C3.HandleInterrupt();                     // Tells the I2C object to execute the next step in the state-machine
}

/* Externally declared in tm4c123gh6pm_startup_ccs.c */
extern "C" void Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);         // Clear the timer interrupt so it can be called again later

    leds.toggleAll();
    imu.peripheral.setTriggerFlag(true);
    depthSensor.peripheral.setTriggerFlag(true);
    headingPID.setTriggerFlag(true);
    depthPID.setTriggerFlag(true);
    camera.pixelPID.setTriggerFlag(true);
    pitchPID.setTriggerFlag(true);
    TiLink.setTriggerFlag(true);
    battery.readTrigger = true;
    motors.forwardTrigger = true;
    motors.rotationTrigger = true;
}
