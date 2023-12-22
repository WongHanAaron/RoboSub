#include "MS5837.hpp"
#include "sensorlib/i2cm_drv.h"
#include "driverlib/sysctl.h"
#include "sensorlib/i2cm_drv.h"
#include <math.h>
#include <string.h>


const float Exodus::MS5837::Pa = 100.0f;
const float Exodus::MS5837::bar = 0.001f;
const float Exodus::MS5837::mbar = 1.0f;

const uint8_t Exodus::MS5837::MS5837_30BA = 0;
const uint8_t Exodus::MS5837::MS5837_02BA = 1;

Exodus::MS5837::MS5837() {
    initialized = false;
    fluidDensity = 1029;
}

bool Exodus::MS5837::initialize(I2C* i2cObject) {
    Timer.initialize();
    if (i2cObject != 0) I2CObject = i2cObject;

    // Reset the MS5837, per datasheet
    // Wire.beginTransmission(MS5837_ADDR);
    // Wire.write(MS5837_RESET);
    // Wire.endTransmission();
    unsigned char value[3];
    value[0] = MS5837_RESET;
    //I2CMWrite(i2cInst, MS5837_ADDR, value, 1, NULL, NULL);
    I2CObject->write(MS5837_ADDR, value, 1);

    // Read calibration values and CRC
    for ( uint8_t i = 0 ; i < 7 ; i++ ) {
        // Wire.beginTransmission(MS5837_ADDR);
        // Wire.write(MS5837_PROM_READ+i*2);
        value[0] = MS5837_PROM_READ+i*2;
        // I2CMWrite(i2cInst, MS5837_ADDR, value, 1, NULL, NULL);
        I2CObject->write(MS5837_ADDR, value, 1);

        // Wire.endTransmission();
        // Wire.requestFrom(MS5837_ADDR,2);
        // I2CMRead(i2cInst, MS5837_ADDR, value, 0, value + 1, 2, NULL, NULL);
        I2CObject->read(MS5837_ADDR, value, 0, value + 1, 2);
        C[i] = (value[1] << 8) | value[2];
    }

    // Verify that data is correct with CRC
    uint8_t crcRead = C[0] >> 12;
    uint8_t crcCalculated = crc4(C);

    if ( crcCalculated == crcRead ) {
        initialized = true;
        return true; // Initialization success
    }

    return false; // CRC fail
}

void Exodus::MS5837::setModel(uint8_t model) {
    model = model;
}

void Exodus::MS5837::setI2C(I2C* i2cObject) {
    I2CObject = i2cObject;
}

void Exodus::MS5837::setFluidDensity(float density) {
    fluidDensity = density;
}

void Exodus::MS5837::read() {
    if (!initialized) return;

    // Request D1 conversion
    // Wire.beginTransmission(MS5837_ADDR);
    // Wire.write(MS5837_CONVERT_D1_8192);
    // Wire.endTransmission();
    unsigned char value[4];

    // I2CObject->debugOutput->printf("Write CONVERT_D1\n~");
    value[0] = MS5837_CONVERT_D1_8192;
    // I2CMWrite(i2cInst, MS5837_ADDR, value, 1, NULL, NULL);
    I2CObject->write(MS5837_ADDR, value, 1);
    SysCtlDelay(SysCtlClockGet()/120);
    // delay(20); // Max conversion time per datasheet

    // Wire.beginTransmission(MS5837_ADDR);
    // Wire.write(MS5837_ADC_READ);
    // Wire.endTransmission();

    // I2CObject->debugOutput->printf("Write ADC_READ\n~");
    value[0] = MS5837_ADC_READ;
    // I2CMWrite(i2cInst, MS5837_ADDR, value, 1, NULL, NULL);
    I2CObject->write(MS5837_ADDR, value, 1);
    // Wire.requestFrom(MS5837_ADDR,3);


    // I2CObject->debugOutput->printf("Read ADDRESS\n~");
    // I2CMRead(i2cInst, MS5837_ADDR, value, 1, value + 1, 3, NULL, NULL);
    I2CObject->read(MS5837_ADDR, value, 0, value + 1, 3);
    D1 = 0;
    D1 = value[1];
    D1 = (D1 << 8) | value[2];
    D1 = (D1 << 8) | value[3];

    // Request D2 conversion;
    // Wire.beginTransmission(MS5837_ADDR);
    // Wire.write(MS5837_CONVERT_D2_8192);
    // Wire.endTransmission();

    // I2CObject->debugOutput->printf("Write CONVERT_D2\n~");
    value[0] = MS5837_CONVERT_D2_8192;
    // I2CMWrite(i2cInst, MS5837_ADDR, value, 1, NULL, NULL);
    I2CObject->write(MS5837_ADDR, value, 1);
    // delay(20); // Max conversion time per datasheet

    SysCtlDelay(SysCtlClockGet()/150);

    // Wire.beginTransmission(MS5837_ADDR);
    // Wire.write(MS5837_ADC_READ);
    // Wire.endTransmission();


    // I2CObject->debugOutput->printf("Write ADC_READ 2\n~");
    value[0] = MS5837_ADC_READ;
    // I2CMWrite(i2cInst, MS5837_ADDR, value, 1, NULL, NULL);
    I2CObject->write(MS5837_ADDR, value, 1);

    // Wire.requestFrom(MS5837_ADDR,3);


    //I2CObject->debugOutput->printf("Read ADDRESS 2\n~");
    // I2CMRead(i2cInst, MS5837_ADDR, value, 0, value + 1, 3, NULL, NULL);
    I2CObject->read(MS5837_ADDR, value, 0, value + 1, 3);
    D2 = 0;
    D2 = value[1];
    D2 = (D2 << 8) | value[2];
    D2 = (D2 << 8) | value[3];

    calculate();
}

void Exodus::MS5837::calculate() {
    // Given C1-C6 and D1, D2, calculated TEMP and P
    // Do conversion first and then second order temp compensation

    int32_t dT = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int32_t SENSi = 0;
    int32_t OFFi = 0;
    int32_t Ti = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    // Terms called
    dT = D2-uint32_t(C[5])*256l;
    if ( model == MS5837_02BA ) {
        SENS = int64_t(C[1])*65536l+(int64_t(C[3])*dT)/128l;
        OFF = int64_t(C[2])*131072l+(int64_t(C[4])*dT)/64l;
        P = (D1*SENS/(2097152l)-OFF)/(32768l);
    } else {
        SENS = int64_t(C[1])*32768l+(int64_t(C[3])*dT)/256l;
        OFF = int64_t(C[2])*65536l+(int64_t(C[4])*dT)/128l;
        P = (D1*SENS/(2097152l)-OFF)/(8192l);
    }

    // Temp conversion
    TEMP = 2000l+int64_t(dT)*C[6]/8388608LL;

    //Second order compensation
    if ( model == MS5837_02BA ) {
        if((TEMP/100)<20){         //Low temp
            // Serial.println("here");
            Ti = (11*int64_t(dT)*int64_t(dT))/(34359738368LL);
            OFFi = (31*(TEMP-2000)*(TEMP-2000))/8;
            SENSi = (63*(TEMP-2000)*(TEMP-2000))/32;
        }
    } else {
        if((TEMP/100)<20){         //Low temp
            Ti = (3*int64_t(dT)*int64_t(dT))/(8589934592LL);
            OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
            SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
            if((TEMP/100)<-15){    //Very low temp
                OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
                SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
            }
        }
        else if((TEMP/100)>=20){    //High temp
            Ti = 2*(dT*dT)/(137438953472LL);
            OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
            SENSi = 0;
        }
    }

    OFF2 = OFF-OFFi;           //Calculate pressure and temp second order
    SENS2 = SENS-SENSi;

    if ( model == MS5837_02BA ) {
        TEMP = (TEMP-Ti);
        P = (((D1*SENS2)/2097152l-OFF2)/32768l)/100;
    } else {
        TEMP = (TEMP-Ti);
        P = (((D1*SENS2)/2097152l-OFF2)/8192l)/10;
    }
}

float Exodus::MS5837::pressure(float conversion) {
    return P*conversion;
}

float Exodus::MS5837::temperature() {
    return TEMP/100.0f;
}

float Exodus::MS5837::depth() {
    return (pressure(MS5837::Pa)-101300)/(fluidDensity*9.80665);
}

float Exodus::MS5837::altitude() {
    return (1-pow((pressure()/1013.25),.190284))*145366.45*.3048;
}

uint8_t Exodus::MS5837::crc4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;

    for ( uint8_t i = 0 ; i < 16; i++ ) {
        if ( i%2 == 1 ) {
            n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
        } else {
            n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
        }
        for ( uint8_t n_bit = 8 ; n_bit > 0 ; n_bit-- ) {
            if ( n_rem & 0x8000 ) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = ((n_rem >> 12) & 0x000F);

    return n_rem ^ 0x00;
}

void Exodus::MS5837::test() {
//gives menu when prompted so we can select specific test
//makes variables true based on if statement and if those are true print the specific test they relate to in the while loop continuously
    I2CObject->debugOutput->printf("[MCU] Depth Sensor Test\n~");  //creates menu
    I2CObject->debugOutput->printf("Sensor Test Menu\n~");
    I2CObject->debugOutput->printf("1. All\n~");
    I2CObject->debugOutput->printf("2. Depth\n~");
    I2CObject->debugOutput->printf("3. Pressure\n~");
    I2CObject->debugOutput->printf("4. Temperature\n~");

    I2CObject->debugOutput->resetBuffer();

    while(!I2CObject->debugOutput->foundCommand);  //waits until gets command

    char testDepth;
    char testPressure;
    char testTemperature;

    testDepth = false;
    testPressure = false;
    testTemperature = false;

    if(strcmp(I2CObject->debugOutput->input, "1") == 0 ){  //sets variables true/false depending on command
        testDepth = true;
        testPressure = true;
        testTemperature = true;
    }
    if(strcmp(I2CObject->debugOutput->input, "2") == 0 ){
        testDepth = true;
    }

    if(strcmp(I2CObject->debugOutput->input, "3") == 0 ){
        testPressure = true;
    }
    if(strcmp(I2CObject->debugOutput->input, "4") == 0 ){
        testTemperature = true;
    }

    if (testDepth == false && testPressure == false && testTemperature == false){
        I2CObject->debugOutput->printf("[MCU] Invalid choice. Accepted values are 1,2,3,4\n~");
    }
    else while(I2CObject->debugOutput->foundCommand) //outputs sensor data based on which values are true/false
    {
        // I2CObject->debugOutput->printf("[MCU] Sending Read Message\n~");
        read();
        I2CObject->debugOutput->printf("[MCU] Read Successful\n~");

        if(testDepth == true){
            I2CObject->debugOutput->printf("[MCU] Depth is: ");
            I2CObject->debugOutput->printf("%f~\n", depth());
        }
        if(testPressure == true){
            I2CObject->debugOutput->printf("[MCU] Pressure is: ");
            I2CObject->debugOutput->printf("%f~\n", pressure());
        }
        if(testTemperature == true){
            I2CObject->debugOutput->printf("[MCU] Temperature is: ");
            I2CObject->debugOutput->printf("%f~\n", temperature());
        }

        Timer.delayBy1ms(100);
        I2CObject->debugOutput->printf("\n~");
    }
    I2CObject->debugOutput->printf("Leave Depth Test\n~");
}

bool Exodus::MS5837::isInitialized(){
    return initialized;
}
