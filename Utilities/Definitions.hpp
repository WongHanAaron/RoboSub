/**
 * File that defines various enums and other c++ statements that
 * are used by a variety of files
 */

#include <stdint.h>
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#define Clk80MHz SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ

const float MINIBOT_DEADBAND_DUTY = 10.0;

enum motor_characteristics {
    SUB_MOTOR_ZERO = 1500,
    SUB_MOTOR_RANGE = 300,
    SUB_MOTOR_DEADBAND_HALF = 30,
    MINIBOT_RANGE = 800,
    MINIBOT_MIN_WIDTH = 1200
};

enum direction {
    input,
    output
};

enum pin {
    PA0 =  0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
    PB0 =  8, PB1, PB2, PB3, PB4, PB5, PB6, PB7,
    PC0 = 16, PC1, PC2, PC3, PC4, PC5, PC6, PC7,
    PD0 = 24, PD1, PD2, PD3, PD4, PD5, PD6, PD7,
    PE0 = 32, PE1, PE2, PE3, PE4, PE5,
    PF0 = 40, PF1, PF2, PF3, PF4
};


enum gpio_state {
    low,
    high
};

enum adc_base {
    BASE_0,
    BASE_1
};

enum UARTOptions {
    UART0A,
    UART1B,
    UART1C,
    UART2D,
    UART3C,
    UART4C,
    UART5E,
    UART6D,
    UART7E
};

enum I2COptions {
    I2C0PB,
    I2C1PA,
    I2C2PE,
    I2C3PD
};

enum led_option {
    RED,
    GREEN,
    BLUE
};

enum PWMPinOptions {
    PB6M0G0,
    PB7M0G0,
    PB4M0G1,
    PB5M0G1,
    PE4M0G2,
    PE5M0G2,
    PC4M0G3,
    PD0M0G3,
    PC5M0G3,
    PD1M0G3,
    PD0M1G0,
    PD1M1G0,
    PA6M1G1,
    PE4M1G1,
    PA7M1G1,
    PE5M1G1,
    PF0M1G2,
    PF1M1G2,
    PF2M1G3,
    PF3M1G3
};

enum JetsonLinkProtocol {
    DES_HEADING = 1,
    DES_DEPTH = 2,
    DES_FORWARD = 3,
    DES_ROTATION = 4,
    DES_PITCH = 7,
    BIG_LIGHT = 8,
    SERVO = 9,
    DEPTH_P = 10,
    DEPTH_I = 11,
    DEPTH_D = 12,
    HEADING_P = 13,
    HEADING_I = 14,
    HEADING_D = 15,
    PIXEL_P = 16,
    PIXEL_I = 17,
    PIXEL_D = 18,
    PITCH_P = 25,
    PITCH_I = 26,
    PITCH_D = 27,
    PID_MODE = 20,
    DEBUG_MODE = 21,
    VERTICAL_PWM = 30,
    ROTATION_PWM = 31,
    PITCH_PWM = 32,
    CUR_BATTERY = 33,
    CUR_HEADING = 34,
    CUR_DEPTH = 35,
    CUR_FORWARD = 36,
    CUR_PITCH = 37,
    CUR_ACCEL_X = 38,
    CUR_ACCEL_Y = 39,
    CUR_ACCEL_Z = 40,
    SET_DEPTH_PID = 43,
    SET_HEADING_PID = 44,
    SET_PIXEL_PID = 45,
    SET_PITCH_PID = 46,
    MOTOR1_PWM = 50,
    MOTOR2_PWM = 51,
    MOTOR3_PWM = 52,
    MOTOR4_PWM = 53,
    MOTOR5_PWM = 54,
    MOTOR6_PWM = 55,
    MOTOR7_PWM = 56,
    MOTOR8_PWM = 57,
    CAM_CENTER_REQ = 68,
    PIXEL_SAMPLE = 69,
    CAM_CENTER_X = 70,
    CAM_CENTER_Y = 71,
    AUTO_CONNECT = 99,
    CAL_MAG_X = 100,
    CAL_MAG_Y = 101,
    TEST = 253,
    SYS_RESET = 254
};

enum BNO055_Addresses {
    BNO055_DEVICE_ADDRESS = 0x28,
    BNO055_OPERATION_MODE = 0x3D,
    BNO055_SYS_TRIGGER = 0x3F,
    BNO055_CHIP_ID = 0x0,
    BNO055_POWER_MODE = 0x3E,
    BNO055_ACCEL_X = 0x8,
    BNO055_ACCEL_Y = 0xA,
    BNO055_ACCEL_Z = 0xC,
    BNO055_MAG_X = 0xE,
    BNO055_MAG_Y = 0x10,
    BNO055_MAG_Z = 0x12,
    BNO055_GYRO_X = 0x14,
    BNO055_GYRO_Y = 0x16,
    BNO055_GYRO_Z = 0x18,
    BNO055_EUL_YAW = 0x1A,
    BNO055_EUL_ROLL = 0x1C,
    BNO055_EUL_PITCH = 0x1E
};

enum PIDInputType {
    Decimal,
    Angle
};

enum MS5837_Address {
    MS5837_ADDR = 0x76,
    MS5837_RESET = 0x1E,
    MS5837_ADC_READ = 0x00,
    MS5837_PROM_READ = 0xA0,
    MS5837_CONVERT_D1_8192 = 0x4A,
    MS5837_CONVERT_D2_8192 = 0x5A
};

#endif
