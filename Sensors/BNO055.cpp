#include "BNO055.hpp"
#include "string.h"
#include "stdio.h"

void Exodus::BNO055::initialize(I2C* i2cObject, unsigned int I2Caddress) {
    // Variable used to return value of communication routine
    uint8_t array[2]; // array[0] = register address
                      // array[1] = data to write/where to place read data (1 Byte)

    XAxisOffset = 0;
    YAxisOffset = 0;

    timer.initialize();
    I2CObject = i2cObject;


    // Set Operation Mode to Normal for configuration
    array[0] = BNO055_OPERATION_MODE;
    array[1] = 0x0;
    I2CObject -> write(BNO055_DEVICE_ADDRESS, array, 2);

    // Reset
    array[0] = BNO055_SYS_TRIGGER;
    array[1] = 0x20;
    I2CObject -> write(BNO055_DEVICE_ADDRESS, array, 2);
    timer.delayBy1s(1);


    //Get Chip ID
    array[0] = BNO055_CHIP_ID;
    do {
        I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 1);
    } while (array[1] != 0xA0);

    // Set to Power Mode to normal operation
    array[0] = BNO055_POWER_MODE;
    array[1] = 0;
    I2CObject -> write(BNO055_DEVICE_ADDRESS, array, 2);

    // Disable Reset
    array[0] = BNO055_SYS_TRIGGER;
    array[1] = 0;
    I2CObject -> write(BNO055_DEVICE_ADDRESS, array, 2);

    // Set External Clock Use (AKA TI Clock over Sensor Clock
    array[0] = BNO055_SYS_TRIGGER;
    array[1] = 0x80;
    I2CObject -> write(BNO055_DEVICE_ADDRESS, array, 2);

    // Set to NDOF Operation mode
    array[0] = BNO055_OPERATION_MODE;
    array[1] = 0xC;
    I2CObject -> write(BNO055_DEVICE_ADDRESS, array, 2);

    timer.delayBy1s(1);
    initialized = true;

}

void Exodus::BNO055::setXAxisOffset(double offset) {
    XAxisOffset = offset;
}

void Exodus::BNO055::setYAxisOffset(double offset) {
    YAxisOffset = offset;
}

double Exodus::BNO055::getXAxisOffset() {
    return XAxisOffset;
}

double Exodus::BNO055::getYAxisOffset() {
    return YAxisOffset;
}

void Exodus::BNO055::Read(float* accel, float* gyro, float* mag, float* eul) {
    uint8_t array[3];

    // Read Accelerometer
    array[0] = BNO055_ACCEL_X;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    accel[0] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    array[0] = BNO055_ACCEL_Y;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    accel[1] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;


    array[0] = BNO055_ACCEL_Z;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    accel[2] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    // Read Magnetometer
    array[0] = BNO055_MAG_X;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    mag[0] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    array[0] = BNO055_MAG_Y;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    mag[1] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;


    array[0] = BNO055_MAG_Z;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    mag[2] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    // Read Gyroscope
    array[0] = BNO055_GYRO_X;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    gyro[0] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    array[0] = BNO055_GYRO_Y;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    gyro[1] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;


    array[0] = BNO055_GYRO_Z;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    gyro[2] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    // read EULER ANGLES (YAW, ROLL, PITCH)
    array[0] = BNO055_EUL_YAW;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    eul[0] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    array[0] = BNO055_EUL_ROLL;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    eul[1] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;


    array[0] = BNO055_EUL_PITCH;
    I2CObject -> read(BNO055_DEVICE_ADDRESS, array, 1, array+1, 2);
    eul[2] = ((int16_t)(array[2] << 8) | (int16_t)array[1])/16.0;

    if (I2CObject->debugOutput != 0 && I2CObject->PrintDebug)
    {
        I2CObject->debugOutput->printf("[MPUDebug] Data read successful\n~");
    }

    mag[1] -= (XAxisOffset);
    mag[0] -= (YAxisOffset);

}

void Exodus::BNO055::read() {
    Read(accel, gyro, mag, eul);
}

void Exodus::BNO055::calibrate() {
    float Mag[3], Accel[3], Gyro[3], Euler[3];
    Read(Accel, Gyro, Mag, Euler);
    XAxisOffset = 0;
    YAxisOffset = 0;

    double xmin = Mag[1];
    double xmax = Mag[1];
    double ymin = Mag[0];
    double ymax = Mag[0];

    char buffer[10];
    I2CObject->debugOutput->resetBuffer();
    while(!I2CObject->debugOutput->foundCommand)
    {
        Read(Accel, Gyro, Mag, Euler);

        I2CObject->debugOutput->printf("\nx: ");
        sprintf(buffer, "%0.3f", Mag[1] * 1e6);
        I2CObject->debugOutput->printf(buffer);
        I2CObject->debugOutput->printf(" y: ");
        sprintf(buffer, "%0.3f", Mag[0] * 1e6);
        I2CObject->debugOutput->printf(buffer);
        I2CObject->debugOutput->printf(" z: ");
        sprintf(buffer, "%0.3f", Mag[2] * 1e6);
        I2CObject->debugOutput->printf(buffer);
        I2CObject->debugOutput->printf("~");

        //find x-min and x-max
        if (Mag[1] < xmin)
            xmin = Mag[1];
        else if (Mag[1] > xmax)
            xmax = Mag[1];

        //find y-min and y-max
        if (Mag[0] < ymin)
            ymin = Mag[0];
        else if (Mag[0] > ymax)
            ymax = Mag[0];
        timer.delayBy1ms(100);
    }
    I2CObject->debugOutput->printf("\nX Min: ");
    sprintf(buffer, "%0.3f", xmin * 1e6);
    I2CObject->debugOutput->printf(buffer);
    I2CObject->debugOutput->printf("~\n");
    I2CObject->debugOutput->printf("X Max: ");
    sprintf(buffer, "%0.3f", xmax * 1e6);
    I2CObject->debugOutput->printf(buffer);
    I2CObject->debugOutput->printf("~\n");
    I2CObject->debugOutput->printf("Y Min: ");
    sprintf(buffer, "%0.3f", ymin * 1e6);
    I2CObject->debugOutput->printf(buffer);
    I2CObject->debugOutput->printf("~\n");
    I2CObject->debugOutput->printf("Y Max: ");
    sprintf(buffer, "%0.3f", ymax * 1e6);
    I2CObject->debugOutput->printf(buffer);
    I2CObject->debugOutput->printf("~\n");

    XAxisOffset = (xmax + xmin)/2;
    YAxisOffset = (ymax + ymin)/2;

    I2CObject->debugOutput->printf("X Axis Offset: ");
    sprintf(buffer, "%0.3f", XAxisOffset*1e6);
    I2CObject->debugOutput->printf(buffer);
    I2CObject->debugOutput->printf("~\n");
    I2CObject->debugOutput->printf("Y Axis Offset: ");
    sprintf(buffer, "%0.3f", YAxisOffset*1e6);
    I2CObject->debugOutput->printf(buffer);
    I2CObject->debugOutput->printf("~\n");
    I2CObject->debugOutput->printf("\nLeaving Calibration.~\n");

    I2CObject->debugOutput->resetBuffer();

}

void Exodus::BNO055::test() {
    I2CObject->debugOutput->printf("[MCU] BNO055 Sensor Test\n~");
    //make menu
    I2CObject->debugOutput->printf("Sensor Test Menu\n~");
    I2CObject->debugOutput->printf("1. all sensors\n~");
    I2CObject->debugOutput->printf("2. Accel\n~");
    I2CObject->debugOutput->printf("3. Gyro\n~");
    I2CObject->debugOutput->printf("4. Mag\n~");
    I2CObject->debugOutput->printf("5. Euler\n~");
    I2CObject->debugOutput->printf("Enter your choice (1, 2, 3, 4, or 5)\n~");

    I2CObject->debugOutput->resetBuffer();

    while(!I2CObject->debugOutput->foundCommand);

    char accel;
    char gyro;
    char mag;
    char eul;

    float Accel[3];
    float Gyro[3];
    float Mag[3];
    float Euler[3];

    accel = false;
    gyro = false;
    mag = false;
    eul = false;

    if(strcmp(I2CObject->debugOutput->input, "1") == 0){
            accel = true;
            gyro = true;
            mag = true;
    }
    if(strcmp(I2CObject->debugOutput->input, "2") == 0){
            accel = true;
    }
    if(strcmp(I2CObject->debugOutput->input, "3") == 0){
            gyro = true;
    }
    if(strcmp(I2CObject->debugOutput->input, "4") == 0){
            mag = true;
    }
    if(strcmp(I2CObject->debugOutput->input, "5") == 0){
            eul = true;
    }

    if (accel == false && gyro == false && mag == false && eul == false){
        I2CObject->debugOutput->printf("[MCU] Invalid Choice, input is 1,2,3,4, or 5~\n");
    }
    else while(I2CObject->debugOutput->foundCommand)
    {
        I2CObject->debugOutput->printf("[MCU] Sending Read Message\n~");
        Read(Accel, Gyro, Mag, Euler);
        I2CObject->debugOutput->printf("[MCU] Read Successful\n~");

        Mag[0] += (5 * 1e-6);
        Mag[1] += (-11.5 * 1e-6);
        Mag[2] += (0 * 1e-6);

        if (accel == true){
            I2CObject->debugOutput->printf("[MCU] Accel x:");
            I2CObject->debugOutput->printf("%f~\n", Accel[0]);
            I2CObject->debugOutput->printf(" Accel y:");
            I2CObject->debugOutput->printf("%f~\n", Accel[1]);
            I2CObject->debugOutput->printf(" Accel z:");
            I2CObject->debugOutput->printf("%f~\n", Accel[2]- 0.4f);
            I2CObject->debugOutput->printf("~\n");
        }

        if (gyro == true){
            I2CObject->debugOutput->printf("[MCU] Gyro x:");
            I2CObject->debugOutput->printf("%f~\n", Gyro[0]);
            I2CObject->debugOutput->printf(" Gyro y:");
            I2CObject->debugOutput->printf("%f~\n", Gyro[1]);
            I2CObject->debugOutput->printf(" Gyro z:");
            I2CObject->debugOutput->printf("%f~\n", Gyro[2]);
            I2CObject->debugOutput->printf("~\n");
        }

        if (mag == true){
            I2CObject->debugOutput->printf("[MCU] Mag x: ");
            I2CObject->debugOutput->printf("%f~\n", Mag[0]);
            I2CObject->debugOutput->printf(" Mag y: ");
            I2CObject->debugOutput->printf("%f~\n", Mag[1]);
            I2CObject->debugOutput->printf(" Mag z: ");
            I2CObject->debugOutput->printf("%f~\n", Mag[2]);
            I2CObject->debugOutput->printf("~\n");
        }

        if (eul == true){
            I2CObject->debugOutput->printf("[MCU] YAW: ");
            I2CObject->debugOutput->printf("%f~\n", Euler[0]);
            I2CObject->debugOutput->printf(" ROLL: ");
            I2CObject->debugOutput->printf("%f~\n", Euler[1]);
            I2CObject->debugOutput->printf(" PITCH: ");
            I2CObject->debugOutput->printf("%f~\n", Euler[2]);
            I2CObject->debugOutput->printf("~\n");
        }


        timer.delayBy1ms(100);
    }
    I2CObject->debugOutput->printf("Leave BNO055 Test\n~");

}

bool Exodus::BNO055::isInitialized() {
    return initialized;
}


