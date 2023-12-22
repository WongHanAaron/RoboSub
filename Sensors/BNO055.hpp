/**
 * IMU LIBRARY
 * The BNO055 is a module that measures various angular changes. Namely, is used for
 * measuring the acceleration of rotation and Yaw (heading rotation).
 */
#include "I2C.hpp"
#include "SysClock.hpp"
#include "SystemDriver.hpp"
#include "Definitions.hpp"

#ifndef SENSORS_BNO055_HPP
#define SENSORS_BNO055_HPP
namespace Exodus {

class BNO055 : public SystemDriver {
public:
    //public methods
    BNO055() : initialized(false) {}
    ~BNO055() {}
    void initialize(I2C* i2cObject, unsigned int I2Caddress = BNO055_DEVICE_ADDRESS);
    void read();
    void calibrate();
    void test();
    bool isInitialized();
    void setXAxisOffset(double offset);
    void setYAxisOffset(double offset);
    double getXAxisOffset();
    double getYAxisOffset();

    float accel[3];
    float gyro[3];
    float mag[3];
    float eul[3];

private:

    I2C* I2CObject;
    int I2Caddress;
    bool initialized;
    SysClock timer;

    // public members
    double XAxisOffset;
    double YAxisOffset;

    void Read(float* accel, float* gyro, float* mag, float* eul);

};

}

#endif /* SENSORS_BNO055_HPP_ */
