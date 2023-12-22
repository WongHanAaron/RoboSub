/**
 * The JetsonLink class is a class the links the TI micro-controller and the Jetson computer over UART communication.
 */
#ifndef JetsonLink_HPP
#define JetsonLink_HPP

#include "UART.hpp"
#include "SystemDriver.hpp"

/**********************************************
 *     TI - RBPI COMMUNICATION PROTOCOL
 **********************************************/

namespace Exodus {

// Received and Sent Values
//#define DES_HEADING     1   // desired heading to be achieved by TI
//#define DES_DEPTH       2   // desired depth to be achieved by TI
//#define DES_FORWARD     3
//#define DES_ROTATION    4
//#define DES_PITCH       7
//
//#define BIG_LIGHT       8
//#define SERVO           9
//
//#define DEPTH_P         10  // Depth proportional tuning parameter
//#define DEPTH_I         11
//#define DEPTH_D         12
//#define HEADING_P       13  // Heading proportional tuning parameter
//#define HEADING_I       14
//#define HEADING_D       15
//#define PIXEL_P         16
//#define PIXEL_I         17
//#define PIXEL_D         18
//#define PITCH_P         25
//#define PITCH_I         26
//#define PITCH_D         27
//
//#define PID_MODE        20  // Sent to toggle PID controls
//#define DEBUG_MODE      21  //
//
//// Sent Values (Received by TiComs)
//#define VERTICAL_PWM    30  // Current PWM value set to both vertically oriented motors
//#define ROTATION_PWM    31
//#define PITCH_PWM       32
//
//#define CUR_BATTERY     33
//#define CUR_HEADING     34  // Current Heading identifier sent with current depth information
//#define CUR_DEPTH       35  // Current Depth identifier sent with current depth information
//#define CUR_FORWARD     36
//#define CUR_PITCH       37
//#define CUR_ACCEL_X      38
//#define CUR_ACCEL_Y      39
//#define CUR_ACCEL_Z      40
//
//// headers to individually toggle PIDs
//#define SET_DEPTH_PID        43
//#define SET_HEADING_PID      44
//#define SET_PIXEL_PID        45
//#define SET_PITCH_PID        46
//
//#define MOTOR1_PWM      50
//#define MOTOR2_PWM      51
//#define MOTOR3_PWM      52
//#define MOTOR4_PWM      53
//#define MOTOR5_PWM      54
//#define MOTOR6_PWM      55
//#define MOTOR7_PWM      56
//#define MOTOR8_PWM      57
//
//#define CAM_CENTER_REQ    68
//#define PIXEL_SAMPLE    69
//#define CAM_CENTER_X    70
//#define CAM_CENTER_Y    71
//
//#define AUTO_CONNECT    99
//
//#define CAL_MAG_X       100
//#define CAL_MAG_Y       101
//
//#define TEST            253
//#define SYS_RESET       254


/************************************************************
 *   RBPI Link Class (Provides functions for communication)
 ***********************************************************/

// Float receiving union
typedef union floatData
{
    float f;
    long l;
}floatConvert;

float charToFloat(char * charArray);
void charToBitStream(char * charArray, char *  returnedStream);

class JetsonLink : public SystemDriver, public UART {
public:
    JetsonLink();

    /**
     * Send a float/convert float to the Jetson via UART
     * @params: identifier indicating the what the float pertains to (identifier)
     *          value to send (value)
     *          string indicating end of transmission (EoT)
     *
     * @return: none
     */
    void sendFloat(JetsonLinkProtocol identifier, float value, const char* EoT = "*&~");

    /**
     * Reset the buffer that is transmitted via UART
     * @params: none
     * @return: none
     */
    void resetBuffer();

    /**
     * Shift the buffer to send the next char
     * @params: number times to shift the buffer (count)
     * @return: none
     */
    void rightShiftBuffer(int count);

    /**
     * Reset the UART Interrupt
     * @params: none
     * @return: none
     */
    void resetInterrupt();
    bool foundCommand;
    char input[50];
    unsigned char incomingByte;
    int inputIndex;

private:
    /**
     * Send a float/convert a float directly to the Jetson via UART
     * @params: value to send (value)
     * @return: none
     */
    void sendFloat(float value);
};





}
#endif /* JetsonLink_HPP */
